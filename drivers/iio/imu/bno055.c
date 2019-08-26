/*
 * BNO055 - Bosch 9-axis orientation sensor
 *
 * Copyright (c) 2016, Intel Corporation.
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * TODO:
 *  - buffering
 *  - interrupt support
 *  - linear and gravitational acceleration (not supported in IIO)
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/acpi.h>	// 2017.10.09:NEB
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>	// 2017.10.09:NEB

#ifndef TRUE
#	define TRUE	1
#endif
#ifndef FALSE
#	define FALSE (!TRUE)
#endif

#define BNO055_DRIVER_NAME				"bno055"

#define BNO055_REG_CHIP_ID				0x00
#define BNO055_REG_PAGE_ID				0x07

#define BNO055_REG_ACC_DATA_X_LSB		0x08
#define BNO055_REG_MAG_DATA_X_LSB		0x0E
#define BNO055_REG_GYR_DATA_X_LSB		0x14
#define BNO055_REG_EUL_HEADING_LSB		0x1A
#define BNO055_REG_QUA_DATA_W_LSB		0x20
#define BNO055_REG_TEMP					0x34
#define BNO055_REG_CAL_STATUS			0x35	// 2016Nov07:NEB

#define BNO055_REG_SYS_STATUS			0x39
#define BNO055_REG_SYS_ERR				0x3A
#define BNO055_REG_UNIT_SEL				0x3B

#define BNO055_REG_OPR_MODE				0x3D
#define BNO055_REG_PWR_MODE				0x3E	// 2016Nov07:NEB
#define BNO055_REG_AXIS_MAP				0x41	// 2016Nov07:NEB
#define BNO055_REG_AXIS_SIGN			0x42

#define BNO055_REG_ACC_OFFSET_X_LSB		0x55
#define BNO055_REG_MAG_OFFSET_X_LSB		0x5B
#define BNO055_REG_GYR_OFFSET_X_LSB		0x61
#define BNO055_REG_MAG_RADIUS_MSB		0x6A

#define BNO055_OPT_PREFIX				""
#define BNO055_OPT_MODE					BNO055_OPT_PREFIX "operation-mode"
#define BNO055_OPT_UNIT_TEMP			BNO055_OPT_PREFIX "unit-temp"
#define BNO055_OPT_CELSIUS				"celsius"
#define BNO055_OPT_FAHRENHEIT			"fahrenheit"
#define BNO055_OPT_UNIT_EULER			BNO055_OPT_PREFIX "unit-euler"
#define BNO055_OPT_DEGREES				"degrees"
#define BNO055_OPT_RADIANS				"radians"
#define BNO055_OPT_UNIT_GYRO			BNO055_OPT_PREFIX "unit-gyro"
#define BNO055_OPT_DPS					"dps"
#define BNO055_OPT_RPS					"rps"
#define BNO055_OPT_UNIT_ACCEL			BNO055_OPT_PREFIX "unit-accel"
#define BNO055_OPT_MPS_SQUARED			"mps2"
#define BNO055_OPT_MILLI_G				"millig"
#define BNO055_OPT_ROT_CONVENTION		BNO055_OPT_PREFIX "rot-convention"
#define BNO055_OPT_WINDOWS				"windows"
#define BNO055_OPT_ANDROID				"android"
#define BNO055_OPT_AXIS_MAP				BNO055_OPT_PREFIX "axis-map"

enum bno055_axis_map
{
	AXIS_MAP_X_AXIS = 0,
	AXIS_MAP_Y_AXIS = 1,
	AXIS_MAP_Z_AXIS = 2,
};

enum bno055_axis_sign
{
	AXIS_SIGN_POSITIVE = 0,
	AXIS_SIGN_NEGATIVE = 1,
};

struct bno055_calibration_profile
{
	s16 AccelerometerOffsetX;
	s16 AccelerometerOffsetY;
	s16 AccelerometerOffsetZ;
	s16 MagnetometerOffsetX;
	s16 MagnetometerOffsetY;
	s16 MagnetometerOffsetZ;
	s16 GyroscopeOffsetX;
	s16 GyroscopeOffsetY;
	s16 GyroscopeOffsetZ;
	u16 AccelerometerRadius;
	u16 MagnetometerRadius;
};

#define CAL_PROFILE_FORMAT	"%hd %hd %hd %hd %hd %hd %hd %hd %hd %hu %hu"

/*
 * The difference in address between the register that contains the
 * value and the register that contains the offset.  This applies for
 * accel, gyro and magn channels.
 */
#define BNO055_REG_OFFSET_ADDR			0x4D

#define BNO055_OPR_MODE_MASK			GENMASK(3, 0)
#define BNO055_AXIS_MAP_MASK			GENMASK(5, 0)
#define BNO055_AXIS_SIGN_MASK			GENMASK(2, 0)

/* Combination of BNO055 and individual chip IDs. */
#define BNO055_CHIP_ID					0x0F32FBA0

#define BNO055_ROT_CONVENTION_WINDOWS	0
#define BNO055_ROT_CONVENTION_ANDROID	BIT(7)
#define BNO055_TEMP_CELSIUS				0
#define BNO055_TEMP_FAHRENHEIT			BIT(4)
#define BNO055_EUL_DEGREES				0
#define BNO055_EUL_RADIANS				BIT(2)
#define BNO055_GYR_DEGREES				0
#define BNO055_GYR_RADIANS				BIT(1)
#define BNO055_ACC_MPSS					0
#define BNO055_ACC_MILLIG				BIT(0)

/*
 * Operation modes.  It is important that these are listed in the order
 * they appear in the datasheet, as an index to this table is used to
 * write the actual bits in the operation config register.
 */
enum bno055_operation_mode
{
	BNO055_MODE_CONFIG,

	/* Non-fusion modes. */
	BNO055_MODE_ACC_ONLY,
	BNO055_MODE_MAG_ONLY,
	BNO055_MODE_GYRO_ONLY,
	BNO055_MODE_ACC_MAG,
	BNO055_MODE_ACC_GYRO,
	BNO055_MODE_MAG_GYRO,
	BNO055_MODE_AMG,

	/* Fusion modes. */
	BNO055_MODE_IMU,
	BNO055_MODE_COMPASS,
	BNO055_MODE_M4G,
	BNO055_MODE_NDOF_FMC_OFF,
	BNO055_MODE_NDOF,

	BNO055_MODE_MAX,
};

/*
 * Number of channels for each operation mode.  See Table 3-3 in the
 * datasheet for a summary of each operation mode.  Each non-config mode
 * also supports a temperature channel.
 */
static const int bno055_num_channels[] =
{
	0, 4, 4, 4, 7, 7, 7, 10,
	/*
	 * In fusion modes, data from the raw sensors is still
	 * available.  Additionally, the linear and gravitational
	 * components of acceleration are available in all fusion modes,
	 * but there are currently no IIO attributes for these.
	 *
	 * Orientation is exposed both as a quaternion multi-value
	 * (meaning a single channel) and as Euler angles (3 separate
	 * channels).
	 */
	11, 11, 11, 14, 14,
};

struct bno055_axis
{
	union
	{
		unsigned int raw;
		struct
		{
			unsigned int x : 2;
			unsigned int y : 2;
			unsigned int z : 2;
		};
	} axis;
	union
	{
		unsigned int raw;
		struct
		{
			unsigned int z : 1;
			unsigned int y : 1;
			unsigned int x : 1;
		};
	} sign;
};

struct bno055_data
{
	struct regmap *regmap;
	enum bno055_operation_mode op_mode;
	struct bno055_axis axis;
	unsigned int unit_temp;
	unsigned int unit_euler;
	unsigned int unit_gyro;
	unsigned int unit_accel;
	unsigned int rot_convention;
};

/*
 * Note: The BNO055 has two pages of registers.  All the addresses below
 * are page 0 addresses.  If the driver ever uses page 1 registers, it
 * is expected to manually switch between pages via the PAGE ID register
 * and make sure that no other transactions happen.  It also cannot use
 * the regmap interface for accessing registers in page 1.
 */
static const struct regmap_range bno055_writable_ranges[] =
{
	regmap_reg_range( BNO055_REG_ACC_OFFSET_X_LSB, BNO055_REG_MAG_RADIUS_MSB ),
	regmap_reg_range( BNO055_REG_OPR_MODE, BNO055_REG_AXIS_SIGN ),
	regmap_reg_range( BNO055_REG_UNIT_SEL, BNO055_REG_UNIT_SEL ),
	/* Listed as read-only in the datasheet, but probably an error. */
	regmap_reg_range( BNO055_REG_PAGE_ID, BNO055_REG_PAGE_ID ),
};

static const struct regmap_access_table bno055_writable_regs =
{
	.yes_ranges = bno055_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE( bno055_writable_ranges ),
};

/* Only reserved registers are non-readable. */
static const struct regmap_range bno055_non_readable_reg_ranges[] =
{
	regmap_reg_range( BNO055_REG_AXIS_SIGN + 1, BNO055_REG_ACC_OFFSET_X_LSB - 1 ),
};

static const struct regmap_access_table bno055_readable_regs =
{
	.no_ranges = bno055_non_readable_reg_ranges,
	.n_no_ranges = ARRAY_SIZE( bno055_non_readable_reg_ranges ),
};

static const struct regmap_range bno055_volatile_reg_ranges[] =
{
	regmap_reg_range( BNO055_REG_ACC_DATA_X_LSB, BNO055_REG_SYS_ERR ),
};

static const struct regmap_access_table bno055_volatile_regs =
{
	.yes_ranges = bno055_volatile_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE( bno055_volatile_reg_ranges ),
};

static const struct regmap_config bno055_regmap_config =
{
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = BNO055_REG_MAG_RADIUS_MSB + 1,
	.cache_type = REGCACHE_RBTREE,

	.wr_table = &bno055_writable_regs,
	.rd_table = &bno055_readable_regs,
	.volatile_table = &bno055_volatile_regs,
};

static int bno055_read_simple_chan( struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask );
static int bno055_read_temp_chan(struct iio_dev *indio_dev, int *val);
static int bno055_read_quaternion( struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int size, int *vals, int *val_len, long mask);
static int bno055_read_raw_multi( struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int size, int *vals, int *val_len, long mask);
static int bno055_init_chip( struct iio_dev *indio_dev );
static int bno055_read_options( struct iio_dev *indio_dev );
static int bno055_option_axis_map( struct iio_dev *indio_dev );
static bool bno055_is_axis_negative( char* axis );
static enum bno055_axis_map get_axis( const char* axis );
static int bno055_parse_axis_map( const char* buf, struct bno055_axis* axis );
static int bno055_set_axis_map( struct iio_dev *indio_dev );
static int bno055_option_unit_temp( struct iio_dev *indio_dev );
static int bno055_option_unit_euler( struct iio_dev *indio_dev );
static int bno055_option_unit_gyro( struct iio_dev *indio_dev );
static int bno055_option_unit_accel( struct iio_dev *indio_dev );
static int bno055_option_rot_convention( struct iio_dev *indio_dev );
static bool bno055_fusion_mode( struct bno055_data *data );
static void bno055_init_simple_channels( struct iio_chan_spec *p, enum iio_chan_type type, u8 address, bool has_offset );
static int bno055_init_channels( struct iio_dev *indio_dev );
static ssize_t show_calibration_status( struct device* dev, struct device_attribute* attr, char* buf );
static ssize_t read_calibration_profile( struct device* dev, struct device_attribute* attr, char* buf );
static ssize_t write_calibration_profile( struct device* dev, struct device_attribute* attr, const char* buf, size_t count );
static ssize_t read_axis_map( struct device* dev, struct device_attribute* attr, char* buf );
static ssize_t show_system_error( struct device* dev, struct device_attribute* attr, char* buf );
static ssize_t show_system_status( struct device* dev, struct device_attribute* attr, char* buf );

static int bno055_read_simple_chan( struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask )
{
	struct bno055_data *data = iio_priv( indio_dev );
	__le16 raw_val;
	int ret;

	switch ( mask )
	{
	case IIO_CHAN_INFO_RAW:
		ret = regmap_bulk_read( data->regmap, chan->address, &raw_val, 2 );
		if ( ret < 0 )
			return ret;
		*val = ( s16 ) le16_to_cpu( raw_val );
		*val2 = 0;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		ret = regmap_bulk_read( data->regmap, chan->address + BNO055_REG_OFFSET_ADDR, &raw_val, 2 );
		if ( ret < 0 )
			return ret;
		*val = ( s16 ) le16_to_cpu( raw_val );
		*val2 = 0;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		switch ( chan->type )
		{
		case IIO_ACCEL:
			/* Table 3-17: 1 m/s^2 = 100 LSB */
			*val2 = 100;
			break;
		case IIO_MAGN:
			/*
			 * Table 3-19: 1 uT = 16 LSB.  But we need
			 * Gauss: 1G = 0.1 uT.
			 */
			*val2 = 160;
			break;
		case IIO_ANGL_VEL:
			/* Table 3-22: 1 Rps = 900 LSB */
			*val2 = 900;
			break;
		case IIO_ROT:
			/* Table 3-28: 1 degree = 16 LSB */
			*val2 = 16;
			break;
		default:
			return -EINVAL;
		}
		return IIO_VAL_FRACTIONAL;
	default:
		return -EINVAL;
	}
}

static int bno055_read_temp_chan(struct iio_dev *indio_dev, int *val)
{
	struct bno055_data *data = iio_priv(indio_dev);
	unsigned int raw_val;
	int ret;

	ret = regmap_read(data->regmap, BNO055_REG_TEMP, &raw_val);
	if (ret < 0)
		return ret;

	/*
	 * Tables 3-36 and 3-37: one byte of data, signed, 1 LSB = 1C.
	 * ABI wants milliC.
	 */
	*val = raw_val * 1000;

	return IIO_VAL_INT;
}

static int bno055_read_quaternion( struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int size, int *vals, int *val_len, long mask)
{
	struct bno055_data *data = iio_priv( indio_dev );
	__le16 raw_vals[ 4 ];
	int i, ret;

	switch ( mask )
	{
	case IIO_CHAN_INFO_RAW:
		if ( size < 4 )
			return -EINVAL;
		ret = regmap_bulk_read( data->regmap, BNO055_REG_QUA_DATA_W_LSB, raw_vals, sizeof( raw_vals ) );
		if ( ret < 0 )
			return ret;
		for ( i = 0; i < 4; i++ )
		{
			vals[ i ] = ( s16 ) le16_to_cpu( raw_vals[ i ] );
		}
		*val_len = 4;
		return IIO_VAL_INT_MULTIPLE;
	case IIO_CHAN_INFO_SCALE:
		/* Table 3-31: 1 quaternion = 2^14 LSB */
		if ( size < 2 )
		{
			return -EINVAL;
		}
		vals[ 0 ] = 1;
		vals[ 1 ] = 1 << 14;
		return IIO_VAL_FRACTIONAL;
	default:
		return -EINVAL;
	}
}

static int bno055_read_raw_multi( struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int size, int *vals, int *val_len, long mask)
{
	switch ( chan->type )
	{
	case IIO_ACCEL:
	case IIO_MAGN:
	case IIO_ANGL_VEL:
		if ( size < 2 )
			return -EINVAL;
		*val_len = 2;
		return bno055_read_simple_chan( indio_dev, chan, &vals[ 0 ], &vals[ 1 ], mask );

	case IIO_TEMP:
		*val_len = 1;
		return bno055_read_temp_chan( indio_dev, &vals[ 0 ] );

	case IIO_ROT:
		/*
		 * Rotation is exposed as either a quaternion or three
		 * Euler angles.
		 */
		if ( chan->channel2 == IIO_MOD_QUATERNION )
			return bno055_read_quaternion( indio_dev, chan, size, vals, val_len, mask );
		if ( size < 2 )
			return -EINVAL;
		*val_len = 2;
		return bno055_read_simple_chan( indio_dev, chan, &vals[ 0 ], &vals[ 1 ], mask );
	default:
		return -EINVAL;
	}
}

static int bno055_init_chip( struct iio_dev *indio_dev )
{
	struct bno055_data *data = iio_priv( indio_dev );
	struct device *dev = regmap_get_device( data->regmap );
	u8 chip_id_bytes[ 6 ];
	u32 chip_id;
	u16 sw_rev;
	int ret;

	ret = bno055_read_options( indio_dev );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to set options: %d", ret );
		return ret;
	}

	/*
	 * Select page 0
	 */
	ret = regmap_write( data->regmap, BNO055_REG_PAGE_ID, 0 );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to switch to register page 0\n" );
		return ret;
	}

	/*
	 * Select configuration mode.
	 */
	ret = regmap_update_bits( data->regmap, BNO055_REG_OPR_MODE, BNO055_OPR_MODE_MASK, BNO055_MODE_CONFIG );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to select configuration mode\n" );
		return ret;
	}

	/*
	 * Configure per options.
	 */

	/*
	 * Axis map
	 */
	ret = bno055_set_axis_map( indio_dev );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to set axis map\n" );
		return ret;
	}

	/*
	 * Configure units to what we care about.  Also configure
	 * rotation convention.  See datasheet Section 4.3.60.
	 */
	ret = regmap_write( data->regmap, BNO055_REG_UNIT_SEL, data->rot_convention | data->unit_temp | data->unit_euler | data->unit_gyro | data->unit_accel );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to set measurement units\n" );
		return ret;
	}

	ret = regmap_bulk_read( data->regmap, BNO055_REG_CHIP_ID, chip_id_bytes, sizeof( chip_id_bytes ) );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to read chip id\n" );
		return ret;
	}

	chip_id = le32_to_cpu( *( u32 * ) chip_id_bytes );
	sw_rev = le16_to_cpu( *( u16 * ) &chip_id_bytes[ 4 ] );

	if ( chip_id != BNO055_CHIP_ID )
	{
		dev_err( dev, "bad chip id; got %08x expected %08x\n", chip_id, BNO055_CHIP_ID );
		return -EINVAL;
	}

	dev_info( dev, "software revision id %04x\n", sw_rev );

	ret = regmap_update_bits( data->regmap, BNO055_REG_OPR_MODE, BNO055_OPR_MODE_MASK, data->op_mode );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to switch operating mode\n" );
		return ret;
	}

	/*
	 * Table 3-6 says transition from CONFIGMODE to any other mode
	 * takes 7ms.
	 */
	udelay( 10 );

	return 0;
}

static int bno055_read_options( struct iio_dev *indio_dev )
{
	struct bno055_data *data = iio_priv( indio_dev );
	struct device *dev = regmap_get_device( data->regmap );
	int ret;
	//dev_info( dev, "looking for option \"%s\"\n", BNO055_OPT_MODE );
	ret = device_property_read_u32( dev, BNO055_OPT_MODE, &data->op_mode );
	if ( ret < 0 )
	{
		//dev_info( dev, "failed to read operation mode, falling back to accel+gyro\n" );
		data->op_mode = BNO055_MODE_ACC_GYRO;
	}
	if ( data->op_mode >= BNO055_MODE_MAX )
	{
		dev_err( dev, "bad operation mode %d\n", data->op_mode );
		return -EINVAL;
	}
	dev_info( dev, "selected mode 0x%02x", data->op_mode );
	ret = bno055_option_axis_map( indio_dev );
	if ( ret < 0 )
	{
		return ret;
	}
	ret = bno055_option_unit_temp( indio_dev );
	if ( ret < 0 )
	{
		return ret;
	}
	ret = bno055_option_unit_euler( indio_dev );
	if ( ret < 0 )
	{
		return ret;
	}
	ret = bno055_option_unit_gyro( indio_dev );
	if ( ret < 0 )
	{
		return ret;
	}
	ret = bno055_option_unit_accel( indio_dev );
	if ( ret < 0 )
	{
		return ret;
	}
	ret = bno055_option_rot_convention( indio_dev );
	if ( ret < 0 )
	{
		return ret;
	}
	return ret;
}

static int bno055_option_axis_map( struct iio_dev *indio_dev )
{
	struct bno055_data *data = iio_priv( indio_dev );
	struct device *dev = regmap_get_device( data->regmap );
	const char* parameter_string = NULL;
	int ret;
	//dev_info( dev, "looking for option \"%s\"\n", BNO055_OPT_AXIS_MAP );
	ret = device_property_read_string( dev, BNO055_OPT_AXIS_MAP, &parameter_string );
	if ( ret < 0 )
	{
		dev_err( dev, "axis map not specified: %d\n", ret );
		return 0;	// not there, not a problem
	}
	dev_info( dev, "axis map: %s\n", parameter_string );
	ret = bno055_parse_axis_map( parameter_string, &data->axis );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to parse option \"%s\"", parameter_string );
		return -EINVAL;
	}
	return 0;
}

static int bno055_option_unit_temp( struct iio_dev *indio_dev )
{
	struct bno055_data *data = iio_priv( indio_dev );
	struct device *dev = regmap_get_device( data->regmap );
	const char* parameter_string = NULL;
	int ret;
	data->unit_temp = BNO055_TEMP_CELSIUS;	// default to Celsius
	//dev_info( dev, "looking for option \"%s\"\n", BNO055_OPT_UNIT_TEMP );
	ret = device_property_read_string( dev, BNO055_OPT_UNIT_TEMP, &parameter_string );
	if ( ret < 0 )
	{
		dev_info( dev, "temperature unit not specified: %d\n", ret );
		return 0;	// not there, not a problem
	}
	dev_info( dev, "temperature unit: %s\n", parameter_string );
	if ( strncmp( parameter_string, BNO055_OPT_CELSIUS, sizeof( BNO055_OPT_CELSIUS ) - 1 ) )
	{
		data->unit_temp = BNO055_TEMP_CELSIUS;
	}
	else if ( strncmp( parameter_string, BNO055_OPT_FAHRENHEIT, sizeof( BNO055_OPT_FAHRENHEIT ) - 1 ) )
	{
		data->unit_temp = BNO055_TEMP_FAHRENHEIT;
	}
	else
	{
		return -EINVAL;
	}
	return 0;
}

static int bno055_option_unit_euler( struct iio_dev *indio_dev )
{
	struct bno055_data *data = iio_priv( indio_dev );
	struct device *dev = regmap_get_device( data->regmap );
	const char* parameter_string = NULL;
	int ret;
	data->unit_euler = BNO055_EUL_DEGREES;	// default to degrees
	//dev_info( dev, "looking for option \"%s\"\n", BNO055_OPT_UNIT_EULER );
	ret = device_property_read_string( dev, BNO055_OPT_UNIT_EULER, &parameter_string );
	if ( ret < 0 )
	{
		dev_info( dev, "Euler unit not specified: %d\n", ret );
		return 0;	// not there, not a problem
	}
	dev_info( dev, "Euler unit: %s\n", parameter_string );
	if ( strncmp( parameter_string, BNO055_OPT_DEGREES, sizeof( BNO055_OPT_DEGREES ) - 1 ) )
	{
		data->unit_euler = BNO055_EUL_DEGREES;
	}
	else if ( strncmp( parameter_string, BNO055_OPT_RADIANS, sizeof( BNO055_OPT_RADIANS ) - 1 ) )
	{
		data->unit_euler = BNO055_EUL_RADIANS;
	}
	else
	{
		return -EINVAL;
	}
	return 0;
}

static int bno055_option_unit_gyro( struct iio_dev *indio_dev )
{
	struct bno055_data *data = iio_priv( indio_dev );
	struct device *dev = regmap_get_device( data->regmap );
	const char* parameter_string = NULL;
	int ret;
	data->unit_gyro = BNO055_GYR_DEGREES;	// default to degrees
	//dev_info( dev, "looking for option \"%s\"\n", BNO055_OPT_UNIT_GYRO );
	ret = device_property_read_string( dev, BNO055_OPT_UNIT_GYRO, &parameter_string );
	if ( ret < 0 )
	{
		dev_info( dev, "gyro unit not specified: %d\n", ret );
		return 0;	// not there, not a problem
	}
	dev_info( dev, "gyro unit: %s\n", parameter_string );
	if ( strncmp( parameter_string, BNO055_OPT_DPS, sizeof( BNO055_OPT_DPS ) - 1 ) )
	{
		data->unit_gyro = BNO055_GYR_DEGREES;
	}
	else if ( strncmp( parameter_string, BNO055_OPT_RPS, sizeof( BNO055_OPT_RPS ) - 1 ) )
	{
		data->unit_gyro = BNO055_GYR_RADIANS;
	}
	else
	{
		return -EINVAL;
	}
	return 0;
}

static int bno055_option_unit_accel( struct iio_dev *indio_dev )
{
	struct bno055_data *data = iio_priv( indio_dev );
	struct device *dev = regmap_get_device( data->regmap );
	const char* parameter_string = NULL;
	int ret;
	data->unit_accel = BNO055_ACC_MPSS;
	//dev_info( dev, "looking for option \"%s\"\n", BNO055_OPT_UNIT_ACCEL );
	ret = device_property_read_string( dev, BNO055_OPT_UNIT_ACCEL, &parameter_string );
	if ( ret < 0 )
	{
		dev_info( dev, "accel unit not specified: %d\n", ret );
		return 0;	// not there, not a problem
	}
	dev_info( dev, "accel unit: %s\n", parameter_string );
	if ( strncmp( parameter_string, BNO055_OPT_MPS_SQUARED, sizeof( BNO055_OPT_MPS_SQUARED ) - 1 ) )
	{
		data->unit_accel = BNO055_ACC_MPSS;
	}
	else if ( strncmp( parameter_string, BNO055_OPT_MILLI_G, sizeof( BNO055_OPT_MILLI_G ) - 1 ) )
	{
		data->unit_accel = BNO055_ACC_MILLIG;
	}
	else
	{
		return -EINVAL;
	}
	return 0;
}

static int bno055_option_rot_convention( struct iio_dev *indio_dev )
{
	struct bno055_data *data = iio_priv( indio_dev );
	struct device *dev = regmap_get_device( data->regmap );
	const char* parameter_string = NULL;
	int ret;
	//dev_info( dev, "looking for option \"%s\"\n", BNO055_OPT_ROT_CONVENTION );
	data->rot_convention = BNO055_ROT_CONVENTION_WINDOWS;
	ret = device_property_read_string( dev, BNO055_OPT_ROT_CONVENTION, &parameter_string );
	if ( ret < 0 )
	{
		dev_info( dev, "rotation convention not specified: %d\n", ret );
		return 0;	// not there, not a problem
	}
	dev_info( dev, "rotation convention: %s\n", parameter_string );
	if ( strncmp( parameter_string, BNO055_OPT_WINDOWS, sizeof( BNO055_OPT_WINDOWS ) - 1 ) )
	{
		data->rot_convention = BNO055_ROT_CONVENTION_WINDOWS;
	}
	else if ( strncmp( parameter_string, BNO055_OPT_ANDROID, sizeof( BNO055_OPT_ANDROID ) - 1 ) )
	{
		data->rot_convention = BNO055_ROT_CONVENTION_ANDROID;
	}
	else
	{
		return -EINVAL;
	}
	return 0;
}

static bool bno055_is_axis_negative( char* axis )
{
	bool negative = false;
	if ( axis[ 0 ] == '+' )
	{
		axis[ 0 ] = axis[ 1 ];
		axis[ 1 ] = '\0';
	}
	else if ( axis[ 0 ] == '-' )
	{
		axis[ 0 ] = axis[ 1 ];
		axis[ 1 ] = '\0';
		negative = true;
	}
	return negative;
}

static enum bno055_axis_map get_axis( const char* axis )
{
	enum bno055_axis_map value;
	switch ( tolower( axis[ 0 ] ) )
	{
	case 'x':
		{
			value = AXIS_MAP_X_AXIS;
		}
		break;
	case 'y':
		{
			value = AXIS_MAP_Y_AXIS;
		}
		break;
	case 'z':
		{
			value = AXIS_MAP_Z_AXIS;
		}
		break;
	default:
		{
			value = ( enum bno055_axis_map ) - 1;
		}
		break;
	}
	return value;
}

static int bno055_set_axis_map( struct iio_dev *indio_dev )
{
	struct bno055_data *data = iio_priv( indio_dev );
	struct device *dev = regmap_get_device( data->regmap );
	/*
	* Axis map
	*/
	int ret = regmap_write( data->regmap, BNO055_REG_AXIS_MAP, data->axis.axis.raw );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to set axis map\n" );
		return ret;
	}
	ret = regmap_write( data->regmap, BNO055_REG_AXIS_SIGN, data->axis.sign.raw );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to set axis sign\n" );
		return ret;
	}
	return ret;
}

static int bno055_parse_axis_map( const char* buf, struct bno055_axis* axis )
{
	struct
	{
		char* x;
		char* y;
		char* z;
	} value;
	const char* delim = " ,\t\n";
	char local_buf[ 16 ];
	char* pbuf = &local_buf[ 0 ];

	strncpy( local_buf, buf, sizeof( local_buf ) );
	local_buf[ 15 ] = '\0';
	value.x = strsep( &pbuf, delim );
	if ( NULL == value.x )
	{
		return -1;
	}
	value.y = strsep( &pbuf, delim );
	if ( NULL == value.y )
	{
		return -1;
	}
	value.z = strsep( &pbuf, delim );
	if ( NULL == value.z )
	{
		return -1;
	}
	axis->sign.x = bno055_is_axis_negative( value.x ) ? AXIS_SIGN_NEGATIVE : AXIS_SIGN_POSITIVE;
	axis->sign.y = bno055_is_axis_negative( value.y ) ? AXIS_SIGN_NEGATIVE : AXIS_SIGN_POSITIVE;
	axis->sign.z = bno055_is_axis_negative( value.z ) ? AXIS_SIGN_NEGATIVE : AXIS_SIGN_POSITIVE;
	axis->axis.x = get_axis( value.x );
	axis->axis.y = get_axis( value.y );
	axis->axis.z = get_axis( value.z );
	return 0;
}

static bool bno055_fusion_mode( struct bno055_data *data )
{
	return data->op_mode >= BNO055_MODE_IMU;
}

static void bno055_init_simple_channels( struct iio_chan_spec *p, enum iio_chan_type type, u8 address, bool has_offset)
{
	int i;
	int mask = BIT( IIO_CHAN_INFO_RAW );

	/*
	 * Section 3.6.5 of the datasheet explains that in fusion modes,
	 * readout from the output registers is already compensated.  In
	 * non-fusion modes, the output offset is exposed separately.
	 */
	if ( has_offset )
		mask |= BIT( IIO_CHAN_INFO_OFFSET );

	for ( i = 0; i < 3; i++ )
	{
		p[ i ] = ( struct iio_chan_spec )
		{
			.type = type,
			.info_mask_separate = mask,
			.info_mask_shared_by_type = BIT( IIO_CHAN_INFO_SCALE ),
			.modified = 1,
			.channel2 = IIO_MOD_X + i,
			/* Each value is stored in two registers. */
			.address = address + 2 * i,
		};
	}
}

static int bno055_init_channels( struct iio_dev *indio_dev )
{
	struct bno055_data *data = iio_priv( indio_dev );
	struct iio_chan_spec *channels, *p;
	bool has_offset = !bno055_fusion_mode( data );

	channels = kmalloc( sizeof( *channels ) * bno055_num_channels[ data->op_mode ], GFP_KERNEL );
	if ( !channels )
	{
		return -ENOMEM;
	}
	p = channels;

	/* Refer to Table 3-3 of the datasheet for operation modes. */

	if ( data->op_mode == BNO055_MODE_ACC_ONLY ||
		data->op_mode == BNO055_MODE_ACC_MAG ||
		data->op_mode == BNO055_MODE_ACC_GYRO ||
		data->op_mode == BNO055_MODE_AMG ||
		/* All fusion modes use the accelerometer. */
		data->op_mode >= BNO055_MODE_IMU )
	{
		bno055_init_simple_channels( p, IIO_ACCEL, BNO055_REG_ACC_DATA_X_LSB, has_offset );
		p += 3;
	}

	if ( data->op_mode == BNO055_MODE_MAG_ONLY ||
		data->op_mode == BNO055_MODE_ACC_MAG ||
		data->op_mode == BNO055_MODE_MAG_GYRO ||
		data->op_mode == BNO055_MODE_AMG ||
		data->op_mode >= BNO055_MODE_COMPASS )
	{
		bno055_init_simple_channels( p, IIO_MAGN, BNO055_REG_MAG_DATA_X_LSB, has_offset );
		p += 3;
	}

	if ( data->op_mode == BNO055_MODE_GYRO_ONLY ||
		data->op_mode == BNO055_MODE_ACC_GYRO ||
		data->op_mode == BNO055_MODE_MAG_GYRO ||
		data->op_mode == BNO055_MODE_AMG ||
		data->op_mode == BNO055_MODE_IMU ||
		data->op_mode == BNO055_MODE_NDOF_FMC_OFF ||
		data->op_mode == BNO055_MODE_NDOF )
	{
		bno055_init_simple_channels( p, IIO_ANGL_VEL, BNO055_REG_GYR_DATA_X_LSB, has_offset );
		p += 3;
	}

	if ( bno055_fusion_mode( data ) )
	{
		/* Euler angles. */
		bno055_init_simple_channels( p, IIO_ROT, BNO055_REG_EUL_HEADING_LSB, false );
		p += 3;

		/* Add quaternion orientation channel. */
		*p = ( struct iio_chan_spec )
		{
			.type = IIO_ROT,
				.info_mask_separate = BIT( IIO_CHAN_INFO_RAW ) |
				BIT( IIO_CHAN_INFO_SCALE ),
				.modified = 1,
				.channel2 = IIO_MOD_QUATERNION,
		};
		p++;
	}

	/* Finally, all modes have a temperature channel. */
	*p = ( struct iio_chan_spec )
	{
		.type = IIO_TEMP,
			.info_mask_separate = BIT( IIO_CHAN_INFO_PROCESSED ),
	};

	indio_dev->channels = channels;
	indio_dev->num_channels = bno055_num_channels[ data->op_mode ];

	return 0;
}

static ssize_t show_calibration_status( struct device* dev, struct device_attribute* attr, char* buf )
{
	union
	{
		unsigned int data;
		struct
		{
			unsigned int magnetometer : 2;
			unsigned int accelerometer : 2;
			unsigned int gyroscope : 2;
			unsigned int system : 2;
		};
	} raw_val;

	struct iio_dev *indio_dev = i2c_get_clientdata( to_i2c_client( dev ) );
	struct bno055_data *data = iio_priv( indio_dev );

	int ret = regmap_read( data->regmap, BNO055_REG_CAL_STATUS, &raw_val.data );
	if ( ret < 0 )
	{
		return ( ssize_t ) ret;
	}
	ret = scnprintf( buf, PAGE_SIZE, "%d %d %d %d\n",
		raw_val.system,
		raw_val.gyroscope,
		raw_val.accelerometer,
		raw_val.magnetometer );
	return ( ssize_t ) ret;
}

static ssize_t show_system_error( struct device* dev, struct device_attribute* attr, char* buf )
{
	union
	{
		unsigned int data;
		u8 error;
	} raw_val;

	struct iio_dev *indio_dev = i2c_get_clientdata( to_i2c_client( dev ) );
	struct bno055_data *data = iio_priv( indio_dev );

	int ret = regmap_read( data->regmap, BNO055_REG_SYS_ERR, &raw_val.data );
	if ( ret < 0 )
	{
		return ( ssize_t ) ret;
	}
	ret = scnprintf( buf, PAGE_SIZE, "%d\n", raw_val.error );
	return ( ssize_t ) ret;
}

static ssize_t show_system_status( struct device* dev, struct device_attribute* attr, char* buf )
{
	union
	{
		unsigned int data;
		u8 status;
	} raw_val;

	struct iio_dev *indio_dev = i2c_get_clientdata( to_i2c_client( dev ) );
	struct bno055_data *data = iio_priv( indio_dev );

	int ret = regmap_read( data->regmap, BNO055_REG_SYS_STATUS, &raw_val.data );
	if ( ret < 0 )
	{
		return ( ssize_t ) ret;
	}
	ret = scnprintf( buf, PAGE_SIZE, "%d\n", raw_val.status );
	return ( ssize_t ) ret;
}

static ssize_t read_calibration_profile( struct device* dev, struct device_attribute* attr, char* buf )
{
	struct iio_dev *indio_dev = i2c_get_clientdata( to_i2c_client( dev ) );
	struct bno055_data *data = iio_priv( indio_dev );
	struct bno055_calibration_profile raw_val;
	int ret = regmap_update_bits( data->regmap, BNO055_REG_OPR_MODE, BNO055_OPR_MODE_MASK, BNO055_MODE_CONFIG );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to select configuration mode\n" );
		return ret;
	}
	ret = regmap_bulk_read( data->regmap, BNO055_REG_ACC_OFFSET_X_LSB, &raw_val, sizeof( raw_val ) );
	if ( ret < 0 )
	{
		return ret;
	}
	ret = regmap_update_bits( data->regmap, BNO055_REG_OPR_MODE, BNO055_OPR_MODE_MASK, data->op_mode );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to set operating mode\n" );
		return ret;
	}
	ret = scnprintf( buf, PAGE_SIZE, CAL_PROFILE_FORMAT "\n",
		raw_val.AccelerometerOffsetX,
		raw_val.AccelerometerOffsetY,
		raw_val.AccelerometerOffsetZ,
		raw_val.MagnetometerOffsetX,
		raw_val.MagnetometerOffsetY,
		raw_val.MagnetometerOffsetZ,
		raw_val.GyroscopeOffsetX,
		raw_val.GyroscopeOffsetY,
		raw_val.GyroscopeOffsetZ,
		raw_val.AccelerometerRadius,
		raw_val.MagnetometerRadius );
	return ret;
}

static ssize_t write_calibration_profile( struct device* dev, struct device_attribute* attr, const char* buf, size_t count )
{
	struct iio_dev *indio_dev = i2c_get_clientdata( to_i2c_client( dev ) );
	struct bno055_data *data = iio_priv( indio_dev );
	struct bno055_calibration_profile raw_val;
	int ret;
	int number_of_fields = sscanf( buf, CAL_PROFILE_FORMAT,
		&raw_val.AccelerometerOffsetX,
		&raw_val.AccelerometerOffsetY,
		&raw_val.AccelerometerOffsetZ,
		&raw_val.MagnetometerOffsetX,
		&raw_val.MagnetometerOffsetY,
		&raw_val.MagnetometerOffsetZ,
		&raw_val.GyroscopeOffsetX,
		&raw_val.GyroscopeOffsetY,
		&raw_val.GyroscopeOffsetZ,
		&raw_val.AccelerometerRadius,
		&raw_val.MagnetometerRadius );
	if ( number_of_fields != sizeof( raw_val ) / sizeof( u16 ) )
	{
		return -number_of_fields;
	}
	ret = regmap_update_bits( data->regmap, BNO055_REG_OPR_MODE, BNO055_OPR_MODE_MASK, BNO055_MODE_CONFIG );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to select configuration mode\n" );
		return ret;
	}
	ret = regmap_bulk_write( data->regmap, BNO055_REG_ACC_OFFSET_X_LSB, &raw_val, sizeof( raw_val ) );
	if ( ret < 0 )
	{
		return ret;
	}
	ret = regmap_update_bits( data->regmap, BNO055_REG_OPR_MODE, BNO055_OPR_MODE_MASK, data->op_mode );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to set operating mode\n" );
		return ret;
	}
	return count;
}

static ssize_t read_axis_map( struct device* dev, struct device_attribute* attr, char* buf )
{
	struct iio_dev *indio_dev = i2c_get_clientdata( to_i2c_client( dev ) );
	struct bno055_data *data = iio_priv( indio_dev );

	struct bno055_axis axis;

	int ret = regmap_read( data->regmap, BNO055_REG_AXIS_MAP, &axis.axis.raw );
	if ( ret < 0 )
	{
		return ( ssize_t ) ret;
	}
	ret = regmap_read( data->regmap, BNO055_REG_AXIS_SIGN, &axis.sign.raw );
	if ( ret < 0 )
	{
		return ( ssize_t ) ret;
	}
	ret = scnprintf( buf, PAGE_SIZE, "%s%c %s%c %s%c\n",
		axis.sign.x ? "-" : "", 'x' + axis.axis.x,
		axis.sign.y ? "-" : "", 'x' + axis.axis.y,
		axis.sign.z ? "-" : "", 'x' + axis.axis.z );
	return ( ssize_t ) ret;
}

static ssize_t write_axis_map( struct device* dev, struct device_attribute* attr, const char* buf, size_t count )
{
	int ret;
	struct iio_dev *indio_dev = i2c_get_clientdata( to_i2c_client( dev ) );
	struct bno055_data *data = iio_priv( indio_dev );
	struct bno055_axis axis;

	/*dev_info( dev, "%s: x=%s%c, y=%s%c, z=%s%c\n",
		__func__,
		data->axis.sign.x ? "-" : "", 'x' + data->axis.axis.x,
		data->axis.sign.y ? "-" : "", 'x' + data->axis.axis.y,
		data->axis.sign.z ? "-" : "", 'x' + data->axis.axis.z );*/
	ret = bno055_parse_axis_map( buf, &axis );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to parse axis definition\n" );
		return ret;
	}
	data->axis = axis;
	ret = regmap_update_bits( data->regmap, BNO055_REG_OPR_MODE, BNO055_OPR_MODE_MASK, BNO055_MODE_CONFIG );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to set configuration mode\n" );
		return ret;
	}
	ret = bno055_set_axis_map( indio_dev );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to set axis map\n" );
		return ret;
	}
	ret = regmap_update_bits( data->regmap, BNO055_REG_OPR_MODE, BNO055_OPR_MODE_MASK, data->op_mode );
	if ( ret < 0 )
	{
		dev_err( dev, "failed to set operating mode\n" );
		return ret;
	}
	return count;
}

static const struct iio_info bno055_info =
{
	.driver_module = THIS_MODULE,
	.read_raw_multi = &bno055_read_raw_multi,
};

static DEVICE_ATTR( cal_status, 0444, show_calibration_status, NULL );
static DEVICE_ATTR( cal_profile, 0644, read_calibration_profile, write_calibration_profile );
static DEVICE_ATTR( axis_map, 0644, read_axis_map, write_axis_map );
static DEVICE_ATTR( system_error, 0444, show_system_error, NULL );
static DEVICE_ATTR( system_status, 0444, show_system_status, NULL );

static int bno055_probe( struct i2c_client *client, const struct i2c_device_id *id )
{
	int ret;
	struct iio_dev *indio_dev;
	struct bno055_data *data;

	indio_dev = devm_iio_device_alloc( &client->dev, sizeof( *data ) );
	if ( !indio_dev )
	{
		return -ENOMEM;
	}
	data = iio_priv( indio_dev );

	data->regmap = devm_regmap_init_i2c( client, &bno055_regmap_config );
	if ( IS_ERR( data->regmap ) )
	{
		return PTR_ERR( data->regmap );
	}
	indio_dev->dev.parent = &client->dev;
	indio_dev->name = BNO055_DRIVER_NAME;
	ret = bno055_init_chip( indio_dev );
	if ( ret )
	{
		return ret;
	}
	ret = bno055_init_channels( indio_dev );
	if ( ret )
	{
		return ret;
	}
	indio_dev->info = &bno055_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	i2c_set_clientdata( client, indio_dev );

	ret = devm_iio_device_register( &client->dev, indio_dev );
	if ( ret < 0 )
	{
		dev_err( &client->dev, "could not register IIO device\n" );
		return ret;
	}

	/*
	calibration status
	*/
	ret = device_create_file( &indio_dev->dev, &dev_attr_cal_status );
	if ( ret ){
		dev_err( &client->dev, "failed to create cal_status sysfs entry.\n" );
		devm_iio_device_unregister( &client->dev, indio_dev );
		return ret;
	}
	/*
	calibration profile
	*/
	ret = device_create_file( &indio_dev->dev, &dev_attr_cal_profile );
	if ( ret ){
		dev_err( &client->dev, "failed to create cal_profile sysfs entry.\n" );
		devm_iio_device_unregister( &client->dev, indio_dev );
		return ret;
	}
	/*
	axis map
	*/
	ret = device_create_file( &indio_dev->dev, &dev_attr_axis_map );
	if ( ret ){
		dev_err( &client->dev, "failed to create axis_map sysfs entry.\n" );
		devm_iio_device_unregister( &client->dev, indio_dev );
		return ret;
	}
	/*
	system error
	*/
	ret = device_create_file( &indio_dev->dev, &dev_attr_system_error );
	if ( ret ){
		dev_err( &client->dev, "failed to create system_error sysfs entry.\n" );
		devm_iio_device_unregister( &client->dev, indio_dev );
		return ret;
	}
	/*
	system status
	*/
	ret = device_create_file( &indio_dev->dev, &dev_attr_system_status );
	if ( ret ){
		dev_err( &client->dev, "failed to create system_status sysfs entry.\n" );
		devm_iio_device_unregister( &client->dev, indio_dev );
		return ret;
	}

	return 0;
}

static int bno055_remove( struct i2c_client *client )
{
	struct iio_dev *indio_dev = i2c_get_clientdata( client );

	device_remove_file( &indio_dev->dev, &dev_attr_cal_status );
	device_remove_file( &indio_dev->dev, &dev_attr_cal_profile );
	device_remove_file( &indio_dev->dev, &dev_attr_axis_map );
	device_remove_file( &indio_dev->dev, &dev_attr_system_error );
	device_remove_file( &indio_dev->dev, &dev_attr_system_status );

	devm_iio_device_unregister( &client->dev, indio_dev );
	return 0;
}

static const struct acpi_device_id bno055_acpi_match[] = {
	{ "bno055", 0 },
	{},
};
MODULE_DEVICE_TABLE( acpi, bno055_acpi_match );

static const struct i2c_device_id bno055_id[] =
{
	{ "bno055", 0 },
	{},
};
MODULE_DEVICE_TABLE( i2c, bno055_id );

static struct i2c_driver bno055_driver =
{
	.driver =
	{
		.name	= BNO055_DRIVER_NAME,
		.acpi_match_table = ACPI_PTR( bno055_acpi_match ),
	},
	.probe		= bno055_probe,
	.remove		= bno055_remove,
	.id_table	= bno055_id,
};

module_i2c_driver( bno055_driver );

MODULE_AUTHOR( "Vlad Dogaru <vlad.dogaru@...>" );
MODULE_DESCRIPTION( "Driver for Bosch BNO055 9-axis orientation sensor" );
MODULE_LICENSE( "GPL v2" );
