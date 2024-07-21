# Motor CAN node ID
int16 nodeid

# Battery voltage read by the driver [V]
float32 voltage

# Electric target current [mA]
float64 targetcurrentma

# Electric target current [A]
float64 targetcurrenta

# Motor current [A]
float32 current

# Error/Fault last code
int32 errorcode

# Error code in readable string format
string errorstring

# Target speed [rpm]
float64 targetspeed

# Current speed [rpm]
float64 actualspeed