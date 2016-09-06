gcc -std=c++14 untether64.mm -o amfistop64 -w -arch armv7 -isysroot "$(xcrun --show-sdk-path --sdk iphoneos)" -framework IOKit -framework Foundation -lz
ldid -Se.xml amfistop64
