xlnx-axienet-drv
================

Xilinx AXI ETHERNET Driver with Shared MII Bus

Add MII bus support, support to controll multiple PHYs through a single MII Bus(MDIO).
--------

			-------------
			|  BCM5464  |
			|           |
			-------------         ----------------------
			|   PHY0    |         |  Zynq7045/Zynq7020 |
			|       MDC |---------|     AXIETH0        | 
			|       MDIO|---------|                    |
			-------------         |                    |
			|   PHY1    |         ----------------------
			|           |
			|           |
			-------------
			|   PHY2    |
			|           |
			|           |
			-------------
			|   PHY3    |
			|           |
			|           |
			-------------


The device tree should be changed as follows to describe the device connections:

			......
					??Your IP NAME??: axi-ethernet@43c00000 {
						axistream-connected = <&axi_dma_0>;
						axistream-control-connected = <&axi_dma_0>;
						clock-frequency = <76923080>;
									compatible = "xlnx,axi-ethernet-buffer-2.0", "xlnx,axi-ethernet-1.00.a";
						device_type = "network";
						interrupt-parent = <&ps7_scugic_0>;
						interrupts = <0 57 4>;
						local-mac-address = [00 0a 35 00 00 01];
						phy-handle = <&phy1>;
						reg = <0x43c00000 0x40000>;
						xlnx,include-mdio = <0x1>;
						xlnx,avb = <0x0>;
						xlnx,halfdup = "1";
						xlnx,include-io = "1";
						xlnx,mcast-extend = <0x0>;
						xlnx,phy-type = <0x3>;
						xlnx,phyaddr = <0x3>;
						xlnx,rxcsum = <0x0>;
						xlnx,rxmem = <0x1000>;
						xlnx,rxvlan-strp = <0x0>;
						xlnx,rxvlan-tag = <0x0>;
						xlnx,rxvlan-tran = <0x0>;
						xlnx,stats = <0x1>;
						xlnx,txcsum = <0x0>;
						xlnx,txmem = <0x1000>;
						xlnx,txvlan-strp = <0x0>;
						xlnx,txvlan-tag = <0x0>;
						xlnx,txvlan-tran = <0x0>;
						xlnx,type = <0x1>;
						mdio {
							#address-cells = <1>;
							#size-cells = <0>;
							
							phy1: phy@1 {
								compatible = "broadcom,bcm5464";
								device_type = "ethernet-phy";
								reg = <1>;
							} ;

							phy2: phy@2 {
								compatible = "broadcom,bcm5464";
								device_type = "ethernet-phy";
								reg = <2>;
							} ;
			
							phy3: phy@3 {
								compatible = "broadcom,bcm5464";
								device_type = "ethernet-phy";
								reg = <3>;
							} ;
			
							phy4: phy@4 {
								compatible = "broadcom,bcm5464";
								device_type = "ethernet-phy" ;
								reg = <4>;
							} ;
						} ;
					} ;
			......


