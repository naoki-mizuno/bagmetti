# Bagmetti

<div style="margin: 0 auto">
  <img src="images/bagmetti.jpg" alt="Bagmetti processing a bag fie"/>
</div>

**BAG** file utility for **ME**ssage, **T**f, and **TI**me.


## What's This?

Bagmetti is a collection of utility scripts for filtering and modifying
existing [bag files](http://wiki.ros.org/Bags). You can use Bagmetti, for
example, when:

- you've done `rosbag record -a` to collected all data in an experiment, but
  want to create a more lightweight bag file with fewer topics for
  post-processing (e.g. without the Velodyne point cloud data)
- running amcl on a bag file that was created while running gmapping (which
  means you need to exclude the `map` to `odom` transformations done by
  gmapping)
- creating a bag file for only a certain time range
- you want to rename existing TF frame names

## But what about...

Yes, there are existing alternatives, but sometimes they aren't good enough.


### `rosbag filter`

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


### `rosbag play -s 42 -u 60 foo.bag`

You can specify what time to start and how long to play. It's useful when you
have multiple time periods that you want to play and don't want to create a
separate bag file for each period, but you would need to write down those time
periods so that you don't forget. If you know you'll be using only some parts
of the bag file, why not create new bag files containing only that time
period?


### `rosbag play foo.bag some_topic:=new_topic_name`

You can rename topic names using the remapping command line arguments or
`<remap from="some_topic" to="new_topic_name" />` in launch files. However,
you can't rename TF transformations with this method. Also, it is something
done on-the-fly when playing bag files, so they don't come along automatically
with the bag file when moving to a different location.


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

Formats are different depending on what script you use (don't worry; they're
really simple and written in YAML).

- [Filter](./docs/filter.md)
- [Rename](./docs/rename.md)

Also check out the [sample config files](./samples).


## TODO

- Unit tests
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
