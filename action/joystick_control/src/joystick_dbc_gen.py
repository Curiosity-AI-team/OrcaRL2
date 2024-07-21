#!/usr/bin/python3

import cantools

msg = cantools.database.can.Message(0x281, "Joystick", 8, [
      cantools.database.can.Signal("Channel_0",  0, 16),
      cantools.database.can.Signal("Channel_1", 16, 16),
      cantools.database.can.Signal("Channel_2", 32, 16),
      cantools.database.can.Signal("Channel_3", 48, 16),
    ])

db = cantools.database.can.Database([
        msg
    ]
)

DBC_FILE_NAME = "joystick.dbc"
cantools.database.dump_file(db, DBC_FILE_NAME)
db = cantools.database.load_file(DBC_FILE_NAME)
print(db)
