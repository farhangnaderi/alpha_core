/*
This node toggles the GPIO on the PI
GPIO 9, 12, 13
To control the Power source on the power distribution board.
*/


#include "gpio_manager.hpp"
#include <iostream> 
#include <string>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <softServo.h>
#include <softPwm.h>

GPIOManager::GPIOManager()
{
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));

    //get regular GPIO names and io number
    m_pnh->getParam("device_name", m_device_name);
    m_pnh->getParam("gpio", m_gpio_id);
    m_gpio_count = m_gpio_id.size();

      ///make pwm vector and create services
    m_pnh->getParam("pwm_name", m_pwm_device);
    m_pnh->getParam("pwm_io", m_pwm_id);
    m_pwm_count = m_pwm_id.size();
    // printf("pwm count=%d", m_pwm_count);

    m_pnh->getParam("pwm_range", m_pwm_range);
    m_pnh->getParam("pwm_clock", m_pwm_clock);
   
    //make the gpio vector and create services
    for (int i = 0; i <m_gpio_count; i++)
    {
        gpio_t g;
        g.id = i;
        g.device_name = m_device_name[i];
        g.gpio_name =  std::to_string(m_gpio_id[i]);
        g.service_name = "gpio_manager/set_power_gpio" + g.gpio_name;

        g.m_set_gpio = m_nh->advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>
                (
                    g.service_name, 
                    std::bind(
                        &GPIOManager::f_cb_srv_set_power,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2,
                        g.id
                    )
                );
        gpio_vector.push_back(g);
    }

    m_get_p_state = m_nh->advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>
    (
        "gpio_manager/get_power_port_status",
        std::bind(
            &GPIOManager::f_cb_srv_get_state,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    m_set_all_p_state = m_nh->advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>
    (
        "gpio_manager/set_all_power_port",
        std::bind(
            &GPIOManager::f_cb_srv_set_power_all,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    // printf("pwm=%d\r\n", m_pwm_id[0]);
    if(m_pwm_count>0)
    {
        //make the gpio vector and create services
        for (int i = 0; i <m_pwm_count; i++)
        {
            pwm_t p;
            p.id = i;
            p.device_name = m_pwm_device[i];
            p.gpio_name =  std::to_string(m_pwm_id[i]);
            p.topic_name = "gpio_manager/set_pwm_gpio" + p.gpio_name;
            p.m_set_pwm = m_nh->subscribe<std_msgs::Float64>(
                        p.topic_name,  10, 
                        std::bind(
                            &GPIOManager::f_cb_set_pwm,
                            this,
                            std::placeholders::_1,
                            p.id
                        )
                    );
            pwm_vector.push_back(p);
        }
    }
    

    f_initialize_gpio();

}

bool GPIOManager::f_initialize_gpio()
{
    wiringPiSetup();
    // for each gpio we do the following
    for (int i=0; i<m_gpio_count; i++)
    {
       pinMode(m_gpio_id[i],OUTPUT);
       digitalWrite(m_gpio_id[i],LOW);
       gpio_vector[i].state = 0;
    }
    if(m_pwm_count>0)
    {
        for (int i=0; i<m_pwm_count; i++)
        {

            softPwmCreate (m_pwm_id [i], 0, m_pwm_range);
            // Example PWM usage
            // pinMode(m_pwm_id[i], PWM_OUTPUT);
            // pwmWrite(m_pwm_id[i], 0); // Example PWM duty cycle


        }
    }
    return true;

}


bool GPIOManager::f_cb_srv_set_power_all(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
     if(req.data)
    {
        for (int i=0; i<m_gpio_count; i++)
        {
        digitalWrite(m_gpio_id[i], HIGH);
        gpio_vector[i].state = 1;
        }
        res.success = 1;
        res.message = "All power ports are enabled";
    }
    else
    {
        res.success = 1;
        res.message = "All power ports are disabled";
    }
    return true;
}

bool GPIOManager::f_cb_srv_set_power(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res, int id) 
{

    if(req.data)
    {
        digitalWrite(m_gpio_id[id], HIGH);
        // printf("%d\r\n",m_gpio_id[id]);
        gpio_vector[id].state = 1;
        res.success = 1;
        res.message = "Port:" + gpio_vector[id].gpio_name + " power enabled";
        return true;
    }
    else
    {
        digitalWrite(m_gpio_id[id], LOW);
        gpio_vector[id].state = 0;
        res.success = 1;
        res.message = "Port:" + gpio_vector[id].gpio_name + " power disabled";
        return true;
    }
    return true;
}


bool GPIOManager::f_cb_srv_get_state(
                        std_srvs::Trigger::Request &req,
                        std_srvs::Trigger::Response &res)
{
    std::string msg = "#GPIO Manager: ";

    for (int i=0; i<m_gpio_count; i++)
    {
        msg = msg + "GPIO-"+ gpio_vector[i].gpio_name + "=" + std::to_string(gpio_vector[i].state) + " | ";
    }
    res.success = 1;
    res.message = msg;
    return true;
}


void GPIOManager::f_cb_set_pwm(const std_msgs::Float64ConstPtr& msg, int id)
{
    
    // if(msg->data>0.0 && msg->data<1024.0)
    // {
        printf("PWM %d is set to %lf\r\n", m_pwm_id[id], msg->data);
        //100 -> 2m //75->1ms?
        // pwmWrite(m_pwm_id[id], (int)msg->data);
        softPwmWrite (m_pwm_id [id], (int)msg->data);
        // softServoWrite (m_pwm_id[id], (int)msg->data);
        // softPwmWrite ( (int) msg->data*1024.0, 0); 
    // }
    // else
    // {
        // printf("Input should be 0 to 1024");
    // }
}


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "gpio_manager");

    GPIOManager i;

    ros::spin();

    return 0;
}
