# yalu
source code of an incomplete ios 8.4.1 jailbreak.

Run the following command to download and try out the jailbreak on a Mac OS X system with XCODE.

```git clone https://github.com/kpwn/yalu && cd yalu && ./run.sh```


===TODO===

1) create headers for all other iDevices
1a) arm64

- J71AP   (iPad4,1)   [iPad Air]
- J72AP   (iPad4,2)   [iPad Air]
- J73AP   (iPad4,3)   [iPad Air]
- J81AP   (iPad5,3)   [iPad Air 2]
- J82AP   (iPad5,4)   [iPad Air 2]
- J85AP   (iPad4,4)   [iPad mini 3G]
- J86AP   (iPad4,5)   [iPad mini 3G]
- J87AP   (iPad4,6)   [iPad mini 3G]
- N51AP   (iPhone6,1) [iPhone 5S]
- N53AP   (iPhone6,2) [iPhone 5S]                         => done
- N61AP               [iPhone 6]
- N56AP               [iPhone 6 Plus]
- N102AP  (iPod6,1)   [iPod touch 6G]

1b) armv7
- K93AP   (iPad2,1)   [iPad 2]
- K94AP   (iPad2,2)   [iPad 2]
- K95AP   (iPad2,3)   [iPad 2]
- K93AAP  (iPad2,4)   [iPad 2]
- J1AP    (iPad3,1)   [iPad 2]
- J2AP    (iPad3,2)   [iPad 3]
- J2AAP   (iPad3,3)   [iPad 3]
- P101AP  (iPad3,4)   [iPad 4]
- P102AP  (iPad3,5)   [iPad 4]
- P103AP  (iPad3,6)   [iPad 4]
- P105AP  (iPad2,5)   [iPad mini 2G]
- P106AP  (iPad2,6)   [iPad mini 2G]
- P107AP  (iPad2,7)   [iPad mini 2G]
- N94AP   (iPhone4,1) [iPhone 4S]
- N41AP   (iPhone5,1) [iPhone 5]
- N42AP   (iPhone5,2) [iPhone 5]
- N78AP   (iPod5,1)   [iPod touch 5G]
- N78aAP  (iPod5,1)   [iPod touch 5G with iSight camera]

2) rework untether to use the header files

3) make "UNTETHER_AMFI" & "UNTETHER_FULL" architecture-independent
