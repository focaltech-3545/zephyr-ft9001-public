how to build this code:
step 1:
intall python 3.10
download Zephyr SDK 0.17.1-rc2
https://github.com/zephyrproject-rtos/sdk-ng/releases

```bash
pip3 install --user -U west
west init -m https://github.com/zephyrproject-rtos/zephyr --mr v4.0.0
west update
pip install pyelftools
```
step 2 use cmd like this
```bash
west -v  build -p -b ft9001_eval C:/Users/yourname/zephyr/zephyr/samples/drivers/uart/echo_bot -- -DZEPHYR_EXTRA_MODULES="C:/Users/yourname/zephyr/zephyr/focaltech" -DBUILD_VERSION=4.0.0
```
