// Copyright 2023
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#define TRACEPOINT_CREATE_PROBES

#define TRACEPOINT_DEFINE
#include "tracetools/perfetto_trace.h"
#include "tracetools/tracetools.h"

#include <chrono>
#include <condition_variable>
#include <fstream>
#include <thread>

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
  // The backends determine where trace events are recorded. For this example we
  // are going to use the system-wide tracing service, so that we can see our
  // app's events in context with system profiling information.
  args.backends = perfetto::kSystemBackend;
  args.enable_system_consumer = false;

  perfetto::Tracing::Initialize(args);
  perfetto::TrackEvent::Register();
}

void DrawPlayer(int player_number) {
  TRACE_EVENT("ros2", "DrawPlayer", "player_number", player_number);
  // Sleep to simulate a long computation.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void DrawGame() {
  TRACE_EVENT("ros2", "DrawGame");
  DrawPlayer(1);
  DrawPlayer(2);
}

}   // namespace

int main()
{
#ifdef TRACETOOLS_PERFETTO_ENABLED
    InitializePerfetto();
    printf("Perfetto tracing init success for ros2.\n");

    Observer observer;
    observer.WaitForTracingStart();

    DrawGame();
    perfetto::TrackEvent::Flush();

    printf("Perfetto tracing end.\n");
    return 0;
#else
    printf("Perfetto tracing disabled through configuration.\n");
    return 1;
#endif
}

