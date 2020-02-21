  This started out as yet another motorized barn door sky tracker. After finding the code way overcomplicated on units 
 I found online I decided to make a simpler one for myself. Then I started adding serial commands and then a bluetooth unit.
 Finally I used a bluetooth serial phone app to make a mobile console for it. - (I'll try to find a way to share that too).
  Trim speeds for each trackmode are stored automatically whenever they are changed. 

  Realizing its potential for general time lapse panning use, this unit now has better speed and direction controls.
 The basic barn door sky tracking functions will remain.

```
  Serial comm format = :LLnnn# commands can come from usb or bluetooth module if installed.
  Commands:
  S0 = stop
  SS = run sidreal
  SL = run lunar
  SO = run solar time
  ST = run time lapse
  SR = high speed return
  T+ = increment speed trim for mode unit is in. This is stored in eprom.
  T- = decrement speed trim for current mode.
  MSnnn = set rewind/max speed to nnn 
  MXnnn = set max steps to nnn
  MR = Reset count to 0
  DF = Motor direction forward
  DR = Motor direction reverse
  GC = Get current position count
  GS = Get current speedtrim value
  
  Parts list:
   28byj-48 stepper motor-  belt pulley attached 
   motor spur and gear from an RC car, or large and small gt2 belt and pulleys with the big gear 
   or pulley shaft hole threaded 1/4-20) 
   18" 1/4-20 threaded rod bent to 12" radius 
   DRV8825 driver board or other stepper controller
   Arduino nano v3.0
   12v battery
   5in piano hinge
      4 safety nuts and washers
   2- 14 inch boards hinged and drilled for bolt at 12"
```
