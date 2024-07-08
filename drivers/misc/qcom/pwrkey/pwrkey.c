#include <linux/module.h>  
#include <linux/platform_device.h>  
#include <linux/slab.h>  
#include <linux/input.h>  
#include <linux/kthread.h>  
#include <linux/semaphore.h>  

struct v_dev{  
	struct platform_device *p_dev;    
	struct input_dev *input;  
};  

struct v_dev *pwrkey_dev = NULL;  

static ssize_t store_pwrkey(struct device *dev, struct device_attribute *attr,  
 const char *buf, size_t count)  
{  
	int ret,val;
	ret = sscanf(buf,"%d",&val);
	if(1 == val){
		input_report_key(pwrkey_dev->input,KEY_POWER, 1);
		input_sync(pwrkey_dev->input);
		input_report_key(pwrkey_dev->input,KEY_POWER, 0);
		input_sync(pwrkey_dev->input);
	}

	return count;  
}  

      
DEVICE_ATTR(pwrkey,0664,NULL,store_pwrkey);  
      
      
static int pwrkey_probe(struct platform_device *pdev)  
{  
	int ret = -1;  
	printk("linson %s \n",__func__);  

	if(pwrkey_dev->p_dev == pdev){  
		printk("linson platform device is same\n");  
	}  

	pwrkey_dev->input = input_allocate_device();  
	if(!(pwrkey_dev->input)){  
		printk("linson %s request input deivce error\n",__func__);  
		goto free_input;  
	}  

	pwrkey_dev->input->name = "pwrkey_input";  
	input_set_capability(pwrkey_dev->input, EV_KEY, KEY_POWER);

	ret = input_register_device(pwrkey_dev->input);  
	if(ret < 0){  
		printk("linson %s register input device error\n",__func__);  
		goto input_register_free;  
	}  

	device_create_file(&pdev->dev,&dev_attr_pwrkey);  
          
        return 0;  
      
    input_register_free:  
        input_free_device(pwrkey_dev->input);  
    free_input:  
        kfree(pwrkey_dev);  
        return ret;  
}  

static struct of_device_id pwrkey_table[] = {
	{.compatible = "hq,hq-pwrkey",},
	{},
};    
  
static struct platform_driver pwrkey_driver = {  
        .probe = pwrkey_probe,  
        .driver = {  
            .owner = THIS_MODULE,  
            .name = "pwrkey_input",
			.of_match_table = pwrkey_table,  
        },  
};  
      
static int __init pwrkey_init(void)  
{  
	int ret =-1;  

	printk("linson %s\n", __func__);  

	pwrkey_dev = kzalloc(sizeof(struct v_dev),GFP_KERNEL);  
	if(pwrkey_dev == NULL){  
		printk("%s alloc memory  error\n",__func__);  
		return -ENOMEM;  
	}  

	ret = platform_driver_register(&pwrkey_driver);  
	if(ret < 0){  
		printk("linson %s register driver error\n",__func__);  
		return ret;  
	}  

	return 0;  
}  
      
static void __exit pwrkey_exit(void)  
{  
	//printk("linson %s\n", __func__);  
		if(pwrkey_dev->input != NULL){  
		input_unregister_device(pwrkey_dev->input);  
	}  
	//printk("%s debug__1\n",__func__);  

	if(pwrkey_dev != NULL){  
		platform_device_unregister(pwrkey_dev->p_dev);  
	}  
	//printk("linson %s debug__2\n",__func__);  

	platform_driver_unregister(&pwrkey_driver);  
	//printk("%s debug__3\n",__func__);  

	kfree(pwrkey_dev);  
	//printk("linson %s debug__4\n",__func__);  
}  

module_init(pwrkey_init);  
module_exit(pwrkey_exit);  

MODULE_LICENSE("GPL");  
MODULE_AUTHOR("xiangchao.zhong"); 

