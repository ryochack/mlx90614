extern crate linux_embedded_hal as hal;
extern crate mlx90614;

use hal::I2cdev;
use mlx90614::Mlx90614;

fn main() {
    let i2c = I2cdev::new("/dev/i2c-1").unwrap();
    let mut mlx90614 = Mlx90614::new(i2c).unwrap();

    let mut id = [0u16; 4];
    id[0] = mlx90614.read(mlx90614::Register::ID0).unwrap();
    id[1] = mlx90614.read(mlx90614::Register::ID0).unwrap();
    id[2] = mlx90614.read(mlx90614::Register::ID0).unwrap();
    id[3] = mlx90614.read(mlx90614::Register::ID0).unwrap();
    println!("{:x}, {:x}, {:x}, {:x}", id[0], id[1], id[2], id[3]);
}