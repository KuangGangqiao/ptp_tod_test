# mdio-tool
=========
This is tool to read and write MII registers from ethernet physicals under linux.
It has been tested with JLSemi PHY's connected via PCIe and should work
with all drivers implementing the mdio ioctls.

# How to build
```shell
make

```
If you want to be able to execute this script from command line without
to be in the right directory you can copy the compiled binary file
to you `/usr/bin/` directory.
For this command you need *root-privileges*:
```shell
make install
```
# How to use:
```shell
1. we can automatic select page to read and write PHY
./mdio-tool [r] [dev] [page] [reg]
Such as:
./mdio-tool r eth0 0x0 0x0

./mdio-tool [w] [dev] [page] [reg] [val]
Such as:
./mdio-tool w eth0 0x0 0x0 0x0140

2 we can direct reading and writing PHY
./mdio-tool [r] [dev] [reg]
./mdio-tool r eth0 0x0

./mdio-tool [w] [dev] [reg] [val]
./mdio-tool w eth0 0x10 0x0
```
Notice: if Operation not permitted, maybe you need add sudo for command.
