#include <GL/glut.h>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

void TryFeeling(const sensor_msgs::Imu& msg)
{
//  ROS_INFO("I feel: [%f-%f-%f-%f]", msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  ROS_INFO("I feel ...");
}

void init(void)
{
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glMatrixMode(GL_PROJECTION);
    glOrtho(-5, 5, -5, 5, 5, 15);
    glMatrixMode(GL_MODELVIEW);
    gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0);

    return;
}

void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(1.0, 0, 0);
    glutWireTeapot(3);

    glutSwapBuffers();
    return;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "MonkeySun");

  ros::NodeHandle n;
  int rate = 10;
  ros::Subscriber sub = n.subscribe("mpu6050", 1000, TryFeeling);

  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(300, 300);
  glutCreateWindow("OpenGL 3D View");
  init();
  glutDisplayFunc(display);
  glutMainLoop();

  ros::spin();

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  ROS_INFO("Start Light Node");

  // Main loop.
  while (n.ok())
  {
    // Run spin function at the beginning of the loop to acquire new data from ROS topics.
    ros::spinOnce();

//    display();

    r.sleep();
  }

  return 0;
}