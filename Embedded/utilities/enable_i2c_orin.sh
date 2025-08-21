echo "=== Jetson Orin Nano I2C Activation ==="

echo "1. Loading I2C modules..."
sudo modprobe i2c-dev
sudo modprobe i2c-bcm2708

echo "2. Adding user to i2c group..."
sudo usermod -a -G i2c $USER

echo "3. Setting I2C device permissions..."
for i in {0..9}; do
    if [ -e "/dev/i2c-$i" ]; then
        sudo chmod 666 /dev/i2c-$i
        echo "   /dev/i2c-$i permissions set"
    fi
done

echo "4. Verifying I2C activation..."
ls -la /dev/i2c-*

echo "5. Scanning I2C devices..."
for i in {0..9}; do
    if [ -e "/dev/i2c-$i" ]; then
        echo "   I2C bus $i scan:"
        sudo i2cdetect -y $i
    fi
done

echo "=== I2C Activation Complete ==="
echo "Please reboot and try again: sudo reboot"
