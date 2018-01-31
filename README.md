# Bagmetti

<div style="margin: 0 auto">
  <img src="images/bagmetti.jpg" alt="Bagmetti processing a bag fie"/>
</div>

**BAG** file utility for **ME**ssage, **T**f, and **TI**me.


## What's This?

Bagmetti is a utility script for filtering and modifying an existing [bag
file](http://wiki.ros.org/Bags) and create a new bag file. You can use
Bagmetti, for example, when:

- you've done `rosbag record -a` to collected all data in an experiment, but
  only use a few topics for post-processing
- you want a lightweight bag file without the Velodyne point cloud topic
- running amcl on a bag file that was created while running gmapping (which
  means you need to exclude the `map` to `odom` transformations done by
  gmapping)
- creating a bag file for only a certain time range (more solid method than
  `rosbag play -s 42 -u 60`)


### But what about `rosbag filter`?

The problem with `rosbag filter` is that it only accepts one argument, which
is a Python expression, for filtering messages out. This quickly becomes a
one huge string full of `and`s and `or`s, and is pretty much unmanageable.

Using `rosbag filter` also makes it harder to track down what was used for
filtering a certain bag file. Depending on your shell configuration the
command history can be lost, and even if you have the command history you may
not be able to tell what expression was used last. What's more, one-line
expressions are hard to read.

Meet Bagmetti, which lets you manage filtering rules in a YAML file.
Therefore, you could put it in the same directory as the bag file and even
version control it if you with to.


## Running

Filter in or out certain messages from a bag file and output to a different bag file.

```
rosrun bagmetti filter.py <bag file path> <output file path> <config file path>
```

Rename topic names and TF transformations.

```
rosrun bagmetti rename.py <bag file path> <output file path> <config file path>
```


## Config File Format

- [Filter](./docs/filter.md)
- [Rename](./docs/rename.md)

Also check out the sample config files in the `samples` directory.


## TODO

- Python expression feature for `filter.py`

```yaml
topic:
  -:
    # Exclude if message matches the expression
    - name: /imu/data
      expr: msg.linear_acceleration.x < 1.5
    - velodyne_points

tf:
  +:
    # Include if transformation matches the expression
    - name: map -> odom
      expr: tf.translation.x > 0.5
```


## License

MIT

## Author

Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)
