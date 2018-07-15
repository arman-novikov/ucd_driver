#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/input.h>
#include <linux/input/adp5589.h>
#include <linux/ctype.h>
#include <linux/jiffies.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/kernel.h>

#ifndef TTY_FLIPBUF_SIZE
#define TTY_FLIPBUF_SIZE 512
#endif

#include <linux/types.h> // or not

#include "WHEREFROM.h"

#define UCD_WATCH_DOG_PERIOD	5000 // in mls

#define UCD_PORT_ADDR		0x64

#define FAN_COMMAND_1		0x3b //requeiers word parameter
#define FAN_INDEX		0xe7 //requeiers byte
#define MFR_ID			0x99 //requeiers 18 bytes
#define READ_FAN_SPEED_1	0x90 //requeiers word parameter

#define UCD_FANS_COUNT  	5 	// indexation: [0 : FUNS_NUM) 
								// for each UCD

#define VERSION			1
#define RESPONSE_SIZE	8

/* 
*  UCD_xxx_ROT macros are % of fans' max rotation capability
*  These values below are supposed to be used via FAN_COMMAND_1
*  available in ucd 90xxx
*/

#define UCD_MIN_ROT 		0x01
#define UCD_DEFAULT_ROT		0x50
#define UCD_MAX_ROT 		0x63

struct ucd_private {
	struct i2c_adapter *adap;
	struct tty_port port;
	unsigned ver;
	unsigned fan[UCD_FANS_COUNT];
	unsigned char watch_dog_flag;
	struct mutex mutex;
};

static struct tty_port_operations ucd_port_ops; 

static int ucd_open(struct tty_struct *tty, struct file *filp)
{
	struct ucd_private *priv;
	int res;

	priv = container_of(tty->port, struct ucd_private, port); 
	res = tty_port_open(tty->port, tty, filp); 

	if (res < 0)
		return res;

	return 0;
}

static void ucd_close(struct tty_struct *tty, struct file *filp)
{
	struct ucd_private *priv;

	priv = container_of(tty->port, struct ucd_private, port);
	tty_port_close(tty->port, tty, filp);
}

static int ucd_choose_fan(struct i2c_adapter *adap,
				unsigned short a_fan_num)
{
	union i2c_smbus_data data_i; //for FAN_INDEX 
	int res;

	if (a_fan_num >= UCD_FANS_COUNT)
		return -EINVAL;

	data_i.byte = a_fan_num;
	res = i2c_smbus_xfer(adap, UCD_PORT_ADDR, 0,
				I2C_SMBUS_WRITE, FAN_INDEX,
				I2C_SMBUS_BYTE_DATA, &data_i);

	return res;
}

static int ucd_read_fan_speed(struct i2c_adapter *adap)
{
	union i2c_smbus_data data_s; //for READ_FAN_SPEED_1
	int res;

	res = i2c_smbus_xfer(adap, UCD_PORT_ADDR, 0,
						I2C_SMBUS_READ,  READ_FAN_SPEED_1,
						I2C_SMBUS_WORD_DATA, &data_s);

	return (res < 0) ? res : (int) data_s.word;
}

static int ucd_set_rotation(struct i2c_adapter *adap,
		 		unsigned short rotation)
{
	union i2c_smbus_data data_c; //for FAN_COMMAND_1
	int res;

	if (rotation > UCD_MAX_ROT)
		rotation = UCD_MAX_ROT;

	if (rotation < UCD_MIN_ROT)
		rotation = UCD_MIN_ROT;

	data_c.word = rotation;
	res = i2c_smbus_xfer(adap, UCD_PORT_ADDR, 0,
				I2C_SMBUS_WRITE, FAN_COMMAND_1,
				I2C_SMBUS_WORD_DATA, &data_c);

	return res;
}

static int ucd_set_rotation_for_all( struct i2c_adapter *adap, unsigned short rotation)
{
	unsigned short i;

	for (i = 0; i < UCD_FANS_COUNT; i++) {
		if(ucd_choose_fan(adap, i))
			continue; // todo: add special exit() if failed
		if(ucd_set_rotation(adap, rotation))
			continue;// todo: add special exit() if failed
	}

	return 0;
}

static void ucd_send_to_user(struct ucd_private *priv,
				const unsigned char *data, int data_size)
{
	unsigned i;

	for (i = 0; i < data_size; i++) {
		if (priv->port.count >= TTY_FLIPBUF_SIZE)
			tty_flip_buffer_push(&priv->port);
		tty_insert_flip_char(&priv->port, data[i], TTY_NORMAL);
	}
	tty_insert_flip_char(&priv->port, '\n', TTY_NORMAL);
	tty_flip_buffer_push(&priv->port);
	
	return;
}

static unsigned short get_R_param(const unsigned char *value)
{
	int res;
	sscanf(value, "%d", &res);

	return (unsigned short) res;
}

static int __ucd_write(struct ucd_private *priv, const unsigned char *buf, int count)
{
	int res;
	unsigned short index, max = UCD_FANS_COUNT, r_param;
	unsigned char cmd, ch, response[RESPONSE_SIZE] = {0};
	const unsigned char *param;

	if (*buf == '\033') 
		buf++;
	ucd_choose_fan(priv->adap, 1); res = ucd_read_fan_speed(priv->adap);
	if (*buf != 'I')
		return -EINVAL;
	
	ch = *(++buf);
	if (ch == 'A') {				// if it was an 'A' code
		index = 0;				//iterate the whole range of possible indexes
	} else {
		sscanf(buf, "%d", &res);		// get the index
		index = (unsigned short)res;
		max = index + 1; 			// iterate up to the given index
	}

	cmd = *(++buf);		// here a command is supposed to be
	param = ++buf; 		// here a parameter for the command above is supposed to be

	for (; index < max; index++) {
		res = ucd_choose_fan(priv->adap, index); // first choose a fan
		if (res)
			return res;
		
		switch(cmd) { // after apply a command
		case 'R':
			r_param = get_R_param(param);
			if (ucd_set_rotation(priv->adap, r_param)) 
				return -EINVAL;
			else
				break;
		case 'S':
			res = ucd_read_fan_speed(priv->adap);
			sprintf(response,"%d", res);
			ucd_send_to_user(priv, response, RESPONSE_SIZE);
			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}

static int ucd_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	struct ucd_private *priv;
	int res;

	priv = container_of(tty->port, struct ucd_private, port);

	mutex_lock(&priv->mutex);
	res = __ucd_write(priv, buf, count);
	if (res == 0)
		priv->watch_dog_flag = 1;
	mutex_unlock(&priv->mutex);

	return count; // always return num of received chars
}

static int ucd_write_room(struct tty_struct *tty)
{
	/* can write any amount actually */
	return PAGE_SIZE; 
}

static struct tty_operations ucd_ops = {
	.open 	= ucd_open,
	.close 	= ucd_close,
	.write 	= ucd_write,
	.write_room = ucd_write_room,
};

static int ucd_register(struct ucd_private *priv, struct i2c_adapter *adap)
{
	priv->adap = adap;

	tty_port_init(&priv->port); //in libs
	priv->port.ops = &ucd_port_ops;
	mutex_init(&priv->mutex); //in libs

	return 0;
}

static void ucd_unregister(struct ucd_private *priv)
{	
	tty_port_destroy(&priv->port); //in libs
}

static struct tty_driver *ucd_driver;
static struct ucd_private ucd_private;
static struct workqueue_struct *ucd_watch_dog_wq;

static void watch_dog(struct work_struct *work)
{
	int res;

	mutex_lock(&ucd_private.mutex);
	ucd_private.watch_dog_flag = 0;
	mutex_unlock(&ucd_private.mutex);

	mdelay(UCD_WATCH_DOG_PERIOD);

	mutex_lock(&ucd_private.mutex);

	if (ucd_private.watch_dog_flag == 0)
		res = ucd_set_rotation_for_all(ucd_private.adap, UCD_DEFAULT_ROT);

	mutex_unlock(&ucd_private.mutex);
	queue_delayed_work(ucd_watch_dog_wq, to_delayed_work(work),
			   msecs_to_jiffies(UCD_WATCH_DOG_PERIOD));	

}

static DECLARE_DELAYED_WORK(watch_dog_work, watch_dog);

int ucd_init(struct i2c_adapter *adap)
{	
	int res;
	struct i2c_msg msg = {
		.addr = UCD_PORT_ADDR,
	};

	if (adap == NULL) {
		printk(KERN_ERR "ucd: i2c adapter is NULL\n");
		return ENODEV;
	}

	/* lookup for the display control port, assume
	   UCD board is here when the port is found */

	if (i2c_transfer(adap, &msg, 1) != 1) {
		printk(KERN_ERR "ucd: board doesn't"
				" response on port %d\n", msg.addr);
		return ENODEV;	/* no board found */
	}

	ucd_driver = tty_alloc_driver(1, TTY_DRIVER_REAL_RAW |
					TTY_DRIVER_UNNUMBERED_NODE);
	if (IS_ERR(ucd_driver)) {
		res = PTR_ERR(ucd_driver);
		printk(KERN_ERR "ucd: error of ucd_driver");
		goto err;
	}

	ucd_driver->driver_name = "UCD Board";
	ucd_driver->name = "ucd";
	ucd_driver->major = 0;	/* auto */
	ucd_driver->type = TTY_DRIVER_TYPE_SERIAL;
	ucd_driver->init_termios = tty_std_termios;
	ucd_driver->init_termios.c_oflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;

	ucd_private.ver = VERSION;
	ucd_private.watch_dog_flag = 0;

	res = ucd_register(&ucd_private, adap);
	if (res < 0 ) {
		printk(KERN_ERR "ucd: error of ucd_register");
		goto put_driver;
	}

	tty_set_operations(ucd_driver, &ucd_ops);
	tty_port_link_device(&ucd_private.port, ucd_driver, 0);
	res = tty_register_driver(ucd_driver);
	if (res < 0) {
		printk(KERN_ERR "ucd: error of tty_register_driver");
		goto unreg_ucd;
	}

	res = ucd_set_rotation_for_all(ucd_private.adap, UCD_DEFAULT_ROT);
	if (res)
		goto unreg_ucd;

	ucd_watch_dog_wq = create_singlethread_workqueue("ucd-watch_dog-wq");
	if (!ucd_watch_dog_wq) {
		res = -ENOMEM;
		goto unreg_ucd;
	}

	/* kickstart watch_dog */
	queue_delayed_work(ucd_watch_dog_wq, &watch_dog_work,
			   msecs_to_jiffies(UCD_WATCH_DOG_PERIOD));

	return 0;

unreg_ucd:
	printk(KERN_ERR "ucd: can't register tty driver,"
		" code %d\n", res);
	ucd_unregister(&ucd_private);
put_driver:
	put_tty_driver(ucd_driver);	
err:
	ucd_driver = NULL;
	return 0;
}

void ucd_exit(void)
{
	if (ucd_driver) {
		tty_unregister_driver(ucd_driver);
		ucd_unregister(&ucd_private);
		put_tty_driver(ucd_driver); 
		ucd_driver = NULL;
	}

	cancel_delayed_work(&watch_dog_work);
	destroy_workqueue(ucd_watch_dog_wq);
}

const char *ucd_types(void)
{
	return "ucd3k";
}

const struct service_driver ucd_service_driver = {
	.init  = ucd_init,
	.exit  = ucd_exit,
	.types = ucd_types,
};
