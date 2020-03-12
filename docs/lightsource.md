## Light Source
**********
A custom light torch was designed and manufactured to comply towards the dimensions of the Franka Emika end-effector gripper. The STL file of the light stick can be found [here](https://imperialcollegelondon.box.com/s/up7vvrtcb66sjumbtdnhm723nokyev2g) and be FDM printed on 0.2mm layer height at 30% infill. 

![](light.png)

The light unit was designed as a hexagonal shape to be held securely on the end effector with a cone tip to direct the light source towards the centre of the unit.

A second iteration was made to allow for the unit to be picked up automatically along with a single pole single throw (SPST) switch attached for the light source to stay on when the end effector gripper is closed.

##### Attachment
To add in the motion of the light stick being picked up, a magnetic strip was designed to be permanently attached on one end of the end effector. Magnets of the opposite polarity were attached on the main light unit to ensure it remains attached on the end effector when the gripper is opened.

##### Circuitry
The internal circuitry of the light unit consists of two 1.5V AA Batteries, connected to an SPST switch and an LED. This would ensure that when the end effector gripper is closed, the light is turned on and vice versa when the gripper is open. This would also remove unwanted bright spots and allow for the possibility of continuous images within the same frame.

![](circuit.png)
**********
###### Links 
Links to purchase external components

1. [AA Batteries](https://www.amazon.co.uk/gp/product/B0043ZUES6/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1)
2. [SPST Switch](https://www.amazon.co.uk/gp/product/B074XDQL7Y/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1)
3. [Magnets](https://www.first4magnets.com/circular-disc-rod-c34/5mm-dia-x-2mm-thick-n35-neodymium-magnet-0-51kg-pull-p6698#ps_0_6899|ps_1_6895)
4. [LED](https://uk.farnell.com/cree/c513a-mss-cw0z0132/led-5mm-12cd-warm-white/dp/2579585?gclid=Cj0KCQjw0pfzBRCOARIsANi0g0vXbvuarY1_37DbssD6mV25EU2mlCkNRYnBznlkkOHyUvLYjEK_sDsaAklZEALw_wcB&gross_price=true&mckv=sBEOFW8rU_dc|pcrid|78108376509|kword||match||plid||slid||product|2579585|pgrid|14406255429|ptaid|aud-387501912413:pla-61859547613|&CMP=KNC-GUK-GEN-SHOPPING-2579585)