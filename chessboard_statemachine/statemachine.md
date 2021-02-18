Statemachine-Ablauf (Test)

UCI:    CMD_INIT
        CMD_NEWGAME

UCI:    CMD_CALC_MOVE
SM:     hit?
nope{
}
yes{
SM:     Path planning for piece removal
Marlin: Move to piece
        Magnet on
        Move to corner
        Magnet off
}

SM:     Path planning
Marlin: Move to source
        Magnet on
        Move to Dest
        Magnet off
        (Move to center)
UCI:    CMD_MOVE

Wiederholung ab CMD_CALC_MOVE



Ablauf (Spieler)
        
UCI:    CMD_INIT
        CMD_NEWGAME

PLR:    Read move (n Nodes)
SM:     Path planning
Marlin: Move to source
        Magnet on
        Move to Dest
        Magnet off
        (Move to center)
UCI:    CMD_MOVE

Wiederholung ab Spielereingabe

