# Affordable robotic arm
This robotic arm is my take on an affordable 3d printed robotic arm. 

It can cost as little as ~$60 per arm, when using .42 Nm Nema 17 stepper motors and is designed to be upgradeable.  It is easy to get .54Nm and .84 Nm stepper motors on amazon, though they cost slightly more. 

I made this because I was annoyed because every option out there for people interested to get into robotics were extremely expensive. It seemed to me there was just not a good option for an affordable, scalable, and simple robotic arm. 

The gear boxes are just -7:1 planatary gearboxes. I am planning on making -8:1 versions as well. The negative just means the output is the opposite direction of the input.

<img width="400" height="413" alt="image" src="https://github.com/user-attachments/assets/c1f768fe-670e-4730-9014-340ab4d1960a" />
<img width="397" height="353" alt="image" src="https://github.com/user-attachments/assets/1aa192d9-af4d-4e0d-a703-5081405aca77" />

This project is still very much a work in progress. I have plan to add two extra joints at the end for wrist movement, and a camera mount and teleop device for AI training but thats all in due time. My current issue is my power supply decided it never wants to turn on again, so we are at a pause until I fix it or get a new one.

This is what I used for motor control. The Arduino is just flashed with the GRBL CNC example code.
<img width="638" height="666" alt="image" src="https://github.com/user-attachments/assets/c774a26e-c027-4bdf-a33c-374b1c069691" />

https://www.amazon.com/DAOKI-Expansion-Arduino-Heatsink-Engraving/dp/B08KFYKKN4/ref=sr_1_7?channelId=500&clpRedir=Y&dib=eyJ2IjoiMSJ9.IRom5U9-uybtpAHm04gl0RDveyz4ApBAfZI7TIns5In24DaFFd-CcqObJPWkXywpX0t31gAwj8FQIM6QSXsvPGERS23B8MwAzbSWdxIlTRcOCUSXQKCVTgwNjJ2LVbp7G4FcLMpwTxyz4B22bKnFO6i4x1r3633dnW_HFQVz7-OytSd6QJRluqejNIsh61z8V0a30vlN4bz6PSQNWE0KqN0vnbx92-q7FUcl3-xljHM.v6Bls46RiwajI_9v4o9KEa6ACMuN2AJASI7GJDpZqA4&dib_tag=se&keywords=arduino%2Bcnc%2Bshield&plpRedirect=mhFallback&qid=1789353170&sr=8-7&th=1
