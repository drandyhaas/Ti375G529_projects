# Ti375G529_projects
Some tests for the Efinix Ti375C529 dev kit

Make sure to use Zadig to choose the libusb-win32 driver for the Ti dev board interfaces 0 and 1, 
then it will show up in the programmer.


For memtest in ti_lpddr4_debug_tools:
<pre>
cmd
cd ti_lpddr4_debug_tools
c:\Efinity\2025.1\bin\setup.bat
python console.py --dev TI375C529
> all
> eff

Efficiency Test :
||==== WRITE ===||==== READ ====||==EFFICIENCY==||== BANDWIDTH ==||=== ERROR ===||
||      50%     ||      50%     ||   82.982 %   ||  76.476 Gbps  ||     1       ||
||      100%    ||      0%      ||   79.627 %   ||  73.384 Gbps  ||     N/A     ||
||      0%      ||      100%    ||   83.388 %   ||  76.850 Gbps  ||     N/A     ||
</pre>

