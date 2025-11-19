# Ti375G529_projects
Some tests for the Efinix Ti375C529 dev kit

Make sure to use Zadig to choose the libusb-win32 driver for the Ti dev board interfaces 0 and 1, 
then it will show up in the programmer.


For memtest in ti_lpddr4_debug_tools:
<pre>
cmd
cd ti_lpddr4_debug_tools
G:\Efinity\2025.1\bin\setup.bat
python console.py --dev TI375C529
> all
> eff

Device: TI375C529
Type: LPDDR4x
Data width: 32
Density: 8G bit (Per Channel)
Frequency: 1440 Mhz
 - 1520/190 fails
 - 1500/187.5 passes read leveling but then mtests fail
 - 1480/185 passes read leveling but then mtests sometimes fail
Rank: 1
Chip Select Map: 0x5
AXI0: 180.00 Mhz
AXI1: 180.00 Mhz

Efficiency Test :
||==== WRITE ===||==== READ ====||==EFFICIENCY==||== BANDWIDTH ==||=== ERROR ===||
||      50%     ||      50%     ||   82.982 %   ||  76.476 Gbps  ||     1       ||
||      100%    ||      0%      ||   79.627 %   ||  73.384 Gbps  ||     N/A     ||
||      0%      ||      100%    ||   83.388 %   ||  76.850 Gbps  ||     N/A     ||
</pre>

