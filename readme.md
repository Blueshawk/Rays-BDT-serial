 This started out as yet another motorized barn door sky tracker I made after finding the code way overcomplicated on units I found online. I then started adding serial commands and then a bluetooth unit. Finally I used a bluetooth serial phone app to make a mobile console for it. - I'll try to find a way to share that too. It sat on a shelf for over a year before being picked up again recently after realizing its potential for general time lapse panning use. 

  Serial comm format = :LLnnn#
  Commands:
  S0 = stop
  SS = run sidreal
  SL = run lunar
  SO = run solar time
  SR = high speed return
  T+ = increment speed trim for mode unit is in.
  T- = decrement speed trim for current mode.
  MSnnn = set max speed to nnn
  MXnnn = set max steps to nnn
  GC Get current position count
  GS Get current speedtrim value

  Parts list:
   28byj-48 stepper motor-  belt pulley attached 
   motor spur and gear from an RC car with main gear shaft hole threaded 1/4-20
   (I ordered gator belt pulleys, but the geared one I made my dad also works well)
   18" 1/4-20 threaded rod bent to 12" radius
   DRV8825 driver board or other stepper controller
   Arduino nano v3.0
   12v battery
   5in piano hinge
      4 safety nuts and washers
   2- 14 inch boards hinged and drilled for bolt at 12"
