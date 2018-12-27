# OpticalFlowBoard
Hardware design and source code for OpticalFlowBoard

We are [Team 11411 Cherry Pi](https://cherrypi.egoslike.us/). We are a FIRST Tech Challenge (FTC) robotics team based in San Diego, California. Over the past several years, one of the biggest challenges we see with the robots is how to determine where the robot is accurately. During our journey, we discovered many ways to approach this problem. One that looked especially promising is using an optical flow sensor similar to what is available for many optical mice. One such product is produced by PixArt, the [PMW-3901 chip](http://www.pixart.com.tw/product_data.asp?product_id=168&productclassify_id=19&productclassify2_id=&productclassify_name=&productclassify2_name=&partnumber=PMW3901MB-TXQT).

This chip is especially attractive as it is designed for robotics/drone usage with the sensor at a distance from the surface. Many mice sensors require a very close distance to surfaces to operate properly. A big disadvantage of this chip for our purposes is that it uses SPI for communication. Within FTC, SPI is not an option for sensor integration. Instead, analog, digital or I2C sensors are allowed.

We then experimented with translating SPI to I2C and created a prototype using these two products:
* The [Butterfly Board](https://www.tindie.com/products/TleraCorp/butterfly-stm32l433-development-board/)
* The [PMW-3901 Sensor Board](https://www.tindie.com/products/onehorse/pmw3901-optical-flow-sensor/)

These products are open source designs so inspired by those designs, some of our students created a new design combining the two boards together into a unified product. We then investigated getting the boards manufactured and eventually got in contact with [Advanced Assembly](http://www.aapcb.com/). When we described our project to them and that this board was being designed and programmed by middle and high school age kids, they were excited to help us out and graciously provided feedback on our design as well as offering to manufacture an initial run of boards for prototyping for free to us.

