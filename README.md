# ros2_tracing

Tracing tools with perfetto for ROS 2.

Device : RB5Gen2 ubuntu22.04
ROS version: ROS2 humble

## Building

As of humble, these instructions also apply to ubuntu or linux; it will not work out-of-the-box. Also, note that tracing using `ros2_tracing` is not supported on non-Linux systems.
We use perfetto instead of LTTng to trace ROS 2.

To enable tracing:

1. Install [perfetto](https://github.com/google/perfetto/releases) (`>=v37.0`) to control tracing and read traces:

[perfetto for linux](./perfetto_tools/perfetto_for_linux.md)

2. Build ros2_tracing:
```
mkdir -p /ros-ws/src
cd /ros-ws/src/
git clone https://github.com/quic-qrb-ros/ros2_tracing.git

cd ../
source /opt/ros/foxy/setup.bash
export HOME=/data
colcon build --cmake-args -DTRACETOOLS_DISABLED=OFF --allow-overriding tracetools
```
3. Source and check that tracing is enabled:
```
source ./install/local_setup.bash or ./install/local_setup.sh
ros2 run tracetools status

run command:
ros2 run tracetools perfetto_trace
```
4. Add init code for perfetto sdk to ROS node which you need trace as below:
for more detail you can check this page: https://perfetto.dev/docs/instrumentation/tracing-sdk#getting-started
```c

#include "perfetto.h"

PERFETTO_DEFINE_CATEGORIES(PERFETTO_CATEGORY(ros2));
PERFETTO_TRACK_EVENT_STATIC_STORAGE();

namespace {

class Observer : public perfetto::TrackEventSessionObserver {
 public:
  Observer() { perfetto::TrackEvent::AddSessionObserver(this); }
  ~Observer() override { perfetto::TrackEvent::RemoveSessionObserver(this); }

  void OnStart(const perfetto::DataSourceBase::StartArgs&) override {
    std::unique_lock<std::mutex> lock(mutex);
    cv.notify_one();
  }

  void WaitForTracingStart() {
    PERFETTO_LOG("Waiting for tracing to start...");
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait(lock, [] { return perfetto::TrackEvent::IsEnabled(); });
    PERFETTO_LOG("Tracing started");
  }

  std::mutex mutex;
  std::condition_variable cv;
};

void InitializePerfetto() {
  perfetto::TracingInitArgs args;
  args.backends = perfetto::kSystemBackend;
  args.enable_system_consumer = false;

  perfetto::Tracing::Initialize(args);
  perfetto::TrackEvent::Register();
}

}   // namespace

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    InitializePerfetto();
    std::cout << "perfetto has been init " << std::endl;
    Observer observer;
    observer.WaitForTracingStart();
    ...
}
```

### Disabling tracing

Alternatively, to build and disable tracing, use `TRACETOOLS_DISABLED`:

```
colcon build --cmake-args -DTRACETOOLS_DISABLED=ON --allow-overriding tracetools
```

## Tracing

The steps above will not lead to trace data being generated, and thus they will have no impact on execution. 
### Trace command

The first option is to use the `perfetto` command.

```
$ cat /data/linux-arm64/trace_config.txt | perfetto --txt -c - -o /data/linux-arm64/perfetto-trace
```

By default, it will enable all ROS tracepoints and kernel tracepoints. The trace will be written to /data/linux-arm64/perfetto-trace. Run the command with `-h` for more information.

### Launch perfetto trace example
```
$ ros2 ros2 run tracetools perfetto_trace
```

See [this example perfetto_trace file](./tracetools/src/perfetto_trace.cpp) for more information.

## Design

See the [design document](./doc/design_ros_2.md).

## Analysis

See [`perfetto ui`](https://ui.perfetto.dev/).
![](doc/img/perfetto_trace_example.png)
