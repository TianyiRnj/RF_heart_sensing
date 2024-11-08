#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

#include <stdint.h>
#include <math.h>

#include <zephyr/drivers/adc.h>

#include <zephyr/sys/mutex.h>

/* Data structure to store sensor data */
struct sensor_data_t {
	uint16_t accel_x;
	uint16_t accel_y;
	uint16_t accel_z;
	uint16_t ppg_value;
	uint32_t timestamp;
};

#define size_sensor_data sizeof(struct sensor_data_t)
/* Maximum queue size based on available heap memory */
#define MAX_QUEUE_SIZE   (CONFIG_HEAP_MEM_POOL_SIZE / size_sensor_data)

struct k_mutex fifo_count_mutex;
int fifo_count = 0;

/* FIFO Queue */
K_FIFO_DEFINE(fifo_queue);

uint16_t convert_sensor_value_to_uint16(const struct sensor_value *sensor_val)
{
	// Convert sensor_value to a float
	float value = sensor_value_to_float(sensor_val);

	// Clamp the input to the range we can represent
	if (value < -32.768f) {
		value = -32.768f;
	}
	if (value > 32.767f) {
		value = 32.767f;
	}

	// Scale and convert to uint16_t
	int16_t fixed_point = (int16_t)(value * 1000); // Scaling by 1000 to retain precision
	return (uint16_t)fixed_point;                  // Interpret the bits as uint16_t
}

// ADC and sensor configurations
#define ADC_NODE DT_NODELABEL(adc) // DT_N_S_soc_S_adc_40007000
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

#define ADC_RESOLUTION 10
#define ADC_CHANNEL    0
#define ADC_PORT       SAADC_CH_PSELP_PSELP_AnalogInput3 // AIN3 ==  A5 on XiaoSense
#define ADC_REFERENCE  ADC_REF_INTERNAL                  // 0.6V
#define ADC_GAIN       ADC_GAIN_1_5 // ADC_REFERENCE*5 = 3.0V, since supply/VDD is 3V on sensor

struct adc_channel_cfg chl0_cfg = {.gain = ADC_GAIN,
				   .reference = ADC_REFERENCE,
				   .acquisition_time = ADC_ACQ_TIME_DEFAULT,
				   .channel_id = ADC_CHANNEL,
#ifdef CONFIG_ADC_NRFX_SAADC
				   .input_positive = ADC_PORT
#endif
};

int16_t myADCbuf[1];

struct adc_sequence sequence = {.channels = BIT(ADC_CHANNEL),
				.buffer = myADCbuf,
				.buffer_size = sizeof(myADCbuf),
				.resolution = ADC_RESOLUTION};

// static int print_samples;
static int lsm6dsl_trig_cnt;

// static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;
#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
static struct sensor_value magn_x_out, magn_y_out, magn_z_out;
#endif
#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
static struct sensor_value press_out, temp_out;
#endif


#ifdef CONFIG_LSM6DSL_TRIGGER
static void lsm6dsl_trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	static struct sensor_value accel_x, accel_y, accel_z;
	static struct sensor_value gyro_x, gyro_y, gyro_z;
#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
	static struct sensor_value magn_x, magn_y, magn_z;
#endif
#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
	static struct sensor_value press, temp;
#endif
	lsm6dsl_trig_cnt++;

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

	/* lsm6dsl gyro */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_x);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
	/* lsm6dsl external magn */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_MAGN_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_X, &magn_x);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_Y, &magn_y);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_Z, &magn_z);
#endif

#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
	/* lsm6dsl external press/temp */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_PRESS);
	sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_AMBIENT_TEMP);
	sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
#endif

	if (fifo_count < MAX_QUEUE_SIZE) {
		// printk("Adding data to FIFO\n");

		/* Allocate memory for new sensor data */
		struct sensor_data_t *data = k_malloc(size_sensor_data);

		/* Simulate sensor data acquisition */
		data->accel_x = convert_sensor_value_to_uint16(&accel_x);
		data->accel_y = convert_sensor_value_to_uint16(&accel_y);
		data->accel_z = convert_sensor_value_to_uint16(&accel_z);

		// float decompressed = (float)(int16_t)data->accel_x / 1000;
		// printf("Compressed: %u, Decompressed: %f\n", data->accel_x,
		// (double)decompressed); printk("accel_x raw values: val1 = %d, val2 = %d\n",
		// accel_x.val1, accel_x.val2);

		/* ADC reading (PPG value) */
		adc_read(adc_dev, &sequence);
		data->ppg_value = myADCbuf[0];

		/* Add timestamp */
		data->timestamp = k_uptime_get_32();

		/* Push data into FIFO */
		k_fifo_put(&fifo_queue, data);

		printk("Timestamp (ms): %u \n", data->timestamp);

		gyro_x_out = gyro_x;
		gyro_y_out = gyro_y;
		gyro_z_out = gyro_z;

#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
		magn_x_out = magn_x;
		magn_y_out = magn_y;
		magn_z_out = magn_z;
#endif

#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
		press_out = press;
		temp_out = temp;
#endif
		if (k_mutex_lock(&fifo_count_mutex, K_FOREVER) == 0) {
			fifo_count += 1;
			k_mutex_unlock(&fifo_count_mutex);
		}
	}
}
#endif

uint8_t double_to_uint8(double input)
{
	// Ensure the input is within the desired range
	if (input < -15.0) {
		input = -15.0;
	}
	if (input > 15.0) {
		input = 15.0;
	}

	// Scale and shift the input from [-15.0, 15.0] to [0, 255]
	input = round(input * 10.0);     // Round to nearest tenth
	input = (input + 150.0);         // Shift range from [-150, 150] to [0, 300]
	input = input * (255.0 / 300.0); // Scale range from [0, 300] to [0, 255]

	return (uint8_t)input;
}

double uint8_to_double(uint8_t input)
{
	// Scale and shift the input from [0, 255] to [-15.0, 15.0]
	double output = input;
	output = output * (300.0 / 255.0); // Scale range from [0, 255] to [0, 300]
	output = (output - 150.0) / 10.0; // Shift range from [0, 300] to [-150, 150] and scale down

	return output;
}

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* Sensor data to be advertised */
// length of 22 elements
static uint8_t sensor_data[] = {0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99,
				0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99};

/* Set Advertisement data */
static struct bt_data ad[] = {BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
			      BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0xaa, 0xfe),
			      BT_DATA(BT_DATA_MANUFACTURER_DATA, sensor_data, sizeof(sensor_data))};

/* Set Scan Response data */
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void bt_ready(int err)
{
	char addr_s[BT_ADDR_LE_STR_LEN];
	bt_addr_le_t addr = {0};
	size_t count = 1;

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		k_msleep(5);
		return;
	}

	printk("Bluetooth initialized\n");

	/* Start advertising */
	err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		k_msleep(5);
		return;
	}

	bt_id_get(&addr, &count);
	bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));

	printk("Beacon started, advertising as %s\n", addr_s);
}

typedef union {
	float input; // assumes sizeof(float) == sizeof(int)
	unsigned int output;
} data_t;

int main(void)
{
	int err;
	k_mutex_init(&fifo_count_mutex);

	if (!device_is_ready(adc_dev)) {
		printk("adc_dev not ready\n");
		k_msleep(5);
		return 0;
	}
	printk("Sensor ready, setting up accelerometer and gyro...\n");

	err = adc_channel_setup(adc_dev, &chl0_cfg);
	if (err != 0) {
		printk("ADC adc_channel_setup failed with err %d\n", err);
		k_msleep(5);
		return 0;
	}

	printk("Starting Beacon Demo\n");

	/* Initialize the Bluetooth Subsystem */
	// printk("Initializing Bluetooth...\n");
	err = bt_enable(bt_ready);
	// printk("Bluetooth initialization complete, err = %d\n", err);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		k_msleep(5);
		return 0;
	}

	// int cnt = 0;
	// char out_str[64];
	struct sensor_value odr_attr;
	const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

	if (!device_is_ready(lsm6dsl_dev)) {
		printk("sensor: device not ready.\n");
		k_msleep(5);
		return 0;
	}

	/* set accel/gyro sampling frequency to 104 Hz */
	odr_attr.val1 = 416;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY,
			    &odr_attr) < 0) {
		printk("Cannot set sampling frequency for accelerometer.\n");
		k_msleep(5);
		return 0;
	}

	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY,
			    &odr_attr) < 0) {
		printk("Cannot set sampling frequency for gyro.\n");
		k_msleep(5);
		return 0;
	}

#ifdef CONFIG_LSM6DSL_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	if (sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler) != 0) {
		printk("Could not set sensor type and channel\n");
		return 0;
	}
#endif

	if (sensor_sample_fetch(lsm6dsl_dev) < 0) {
		printk("Sensor sample update error\n");
		return 0;
	}

	int num_data_get = 2;
	int index = 0;
	struct sensor_data_t *data = NULL;
	int32_t timestamp = 0;

	while (1) {
		if (fifo_count <= 0) {
			continue;
		}
		printk("Current fifo_count: %d\n", fifo_count);

		// num_data_get = MIN(fifo_count, 2);

		for (int i = 0; i < num_data_get; i++) {
			/* Read sensor value and update sensor_data */
			/* Pop data from FIFO */
			data = (struct sensor_data_t *)k_fifo_get(&fifo_queue, K_FOREVER);
			index = i * 10;
			/* Erase previous */
			// printk("\0033\014");
			// printk("Process data......");

			if (data == NULL) {
				printk("Error: Memory allocation failed!\n");
				return -1;
			}
			data_t dataX, dataY, dataZ;

			/* Assign flt_accelx to sensor_data[0:3] */
			dataX.input = data->accel_x;
			sensor_data[1 + index] = dataX.output & 0xFF;
			sensor_data[0 + index] = (dataX.output >> 8) & 0xFF;

			/* Assign flt_accely to sensor_data[4:7] */
			dataY.input = data->accel_y;
			sensor_data[3 + index] = dataY.output & 0xFF;
			sensor_data[2 + index] = (dataY.output >> 8) & 0xFF;

			/* Assign flt_accelz to sensor_data[8:11] */
			dataZ.input = data->accel_z;
			sensor_data[5 + index] = dataZ.output & 0xFF;
			sensor_data[4 + index] = (dataZ.output >> 8) & 0xFF;

			// sprintf(out_str, "accel x:%f ms/2 y:%f ms/2 z:%f ms/2",
			// (double)flt_accelx, (double)flt_accely, (double)flt_accelz);
			// printk("%s\n", out_str);

			// printk("HEX accel x:%08x ms/2 y:%08x ms/2 z:%08x ms/2", dataX.output,
			// dataY.output, dataZ.output); printk("%s\n", out_str);

			int32_t my_PPGvalue = data->ppg_value;
			// printk("ADC-value: %d\n", my_PPGvalue);
			// int32_t adc_vref = adc_ref_internal(adc_dev);
			// adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &my_PPGvalue);
			// printk("ADC-voltage: %d mV\n", my_PPGvalue);

			// printk("ADC-value-HEX: %08x \n", my_PPGvalue);
			sensor_data[7 + index] = (my_PPGvalue) & 0xFF;
			sensor_data[6 + index] = (my_PPGvalue >> 8) & 0xFF;

			// Convert timestamp to bytes and add to sensor_data
			if (i == 0) {
				timestamp = data->timestamp; // Get system uptime in ms
				sensor_data[10 + index] = timestamp & 0xFF;
				sensor_data[9 + index] = (timestamp >> 8) & 0xFF;
				sensor_data[8 + index] = (timestamp >> 16) & 0xFF;
				printk("Broadcase Timestamp (ms): %u and Message Timestamp: %u \n",
			       k_uptime_get_32(), timestamp);
			}
			else {
				timestamp = data->timestamp - timestamp; // Get system uptime in ms
				sensor_data[8 + index] = timestamp & 0xFF;
				printk("Broadcase Timestamp (ms): %u and Message Timestamp: %u == %u \n",
			       k_uptime_get_32(), timestamp, sensor_data[8 + index]);
			}

			// Print sensor_data[14] to sensor_data[17] in decimal
			// printk("sensor_data[14:17] in Decimal: %u %u %u %u\n", sensor_data[14],
			// sensor_data[15], sensor_data[16], sensor_data[17]);
			// printk("ADC-value: %d\n", my_PPGvalue);

#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
			/* lsm6dsl external magn */
			sprintf(out_str, "magn x:%f gauss y:%f gauss z:%f gauss",
				sensor_value_to_double(&magn_x_out),
				sensor_value_to_double(&magn_y_out),
				sensor_value_to_double(&magn_z_out));
			printk("%s\n", out_str);
#endif

#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
			/* lsm6dsl external press/temp */
			sprintf(out_str, "press: %f kPa - temp: %f deg",
				sensor_value_to_double(&press_out),
				sensor_value_to_double(&temp_out));
			printk("%s\n", out_str);
#endif

			// printk("loop:%d trig_cnt:%d\n\n", ++cnt, lsm6dsl_trig_cnt);

			// for (int i = 0; i < sizeof(sensor_data); i++) {
			// 	sensor_data[i]++;
			// }

			/* Free memory */
			k_free(data);
		}

		if (k_mutex_lock(&fifo_count_mutex, K_FOREVER) == 0) {
			fifo_count -= num_data_get;
			k_mutex_unlock(&fifo_count_mutex);
		}

		/* Update advertising data */
		ad[2].data = sensor_data;
		ad[2].data_len = sizeof(sensor_data);

		/* Restart advertising to update the advertising packet */
		bt_le_adv_stop();
		bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

		k_msleep(4.5); // TODO: edit ms delay here!
	}

	return 0;
}
