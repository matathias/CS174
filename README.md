# CS174
Repository for our CS174 projects

/*
Goals -
+ Implement objects <- DONE
+ Give objects physics variables <- DONE
+ Implement gravity / acceleration in the redraw function / some "physics" subfunction that updates on redraw
+ Implement object collision / collision physics
 + object collision detection <- DONE
+ Draw "stage" (add walls/invisible floor to test collision/gravity on)
+ Implement interactivity on objects (allow "character" object to have velocity adjusted with keypress reads)
+ (Re-)Implement camera, which would just be arcball or something around PC object <- Basically DONE
+ Try to implement high-end video game physics algorithms linked in paper in original proposal <- PROBABLY IMPORTANT? THIS IS PROBABLY WHAT AL CARES ABOUT

=== Fallback complete^^^ ===

+ Create "hitsparks" on object collision? / particle effects on movement? <- DEALING WITH TONS OF SMALL OBJECTS MIGHT BE WORTH MORE ALGORITHMIC MEAT
+ Draw text on screen for points/lifebar/etc (I know this was EC on the openGL assignment, but I did fog/transparency instead)
+ Create "actual" player character out of several solids linked, kind of like the H-bar with individual animations on the "H" for limbs (this fucker will probably look like Kirby or something unless you care lol)
+ Add movement loops/or idle animation?
+ Implement attack/jump buttons, animations for those, implement "cooldown" on those until animation ends
+ Implement hitboxes (unless we want to just do true collision and have it be like https://www.youtube.com/watch?v=ifino3KF7pk -type games? why not?)
+ Implement enemies/enemy animation
+ Implement enemy spawning (limit spawning) and despawning
+ Implement basic enemy AI 
+ Win/Game over conditions and screens?

=== Way too much free time ===
+ Make graphics look better, texture, etc
