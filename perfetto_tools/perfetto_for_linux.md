# perfetto for linux

## Capturing a trace
Due to Perfetto's service-based architecture, in order to capture a trace, the traced (session daemon) and traced_probes (probes and ftrace-interop daemon) need to be running.

### Download perfetto binaries
1. download perfetto from:
https://github.com/google/perfetto/releases and you need copy below files to perfetto release directory:
[config.txt](./config.txt)
[traced.service](./traced.service)
[traced-probes.service](./traced-probes.service)

2. connect your device with adb and run [insall_perfetto.bat](./install_perfetto.bat) on windows locally

### Capturing a perfetto trace
#### Recording a trace through the Perfetto UI
1. Navigate to  https://ui.perfetto.dev/#!/record/instructions and select Record new trace from the left menu. From this page, select and turn on the data sources you want to include in the trace. 
2. Copy the settings to a TXT file and name it like this: config.txt
3. you can use perfetto with sdk as follows :
```
adb push ./config.txt /data/linux-arm64/
adb shell

#use below command for LE
#######################################
adduser -S traced
addgroup -S traced-consumer
usermod -a -G traced-consumer traced
mkdir -m 755 /run/perfetto
addgroup -S traced
chown traced:traced /run/perfetto
 
systemctl start traced.service
systemctl start traced-probes.service
#######################################
#use below command for LU
#######################################
adduser --home /nonexistent --quiet --system --no-create-home --group traced
addgroup --quiet --system traced-consumer
usermod -a -G traced-consumer traced
mkdir -m 755 /run/perfetto
chown traced:traced /run/perfetto
 
systemctl start traced.service
systemctl start traced-probes.service
#######################################
 
cat /data/linux-arm64/config.txt | perfetto --txt -c - -o /data/linux-arm64/perfetto-trace
```
## Visualizing the trace
We can now explore the captured trace visually by using a dedicated web-based UI.

https://ui.perfetto.dev

## Reference
https://perfetto.dev/docs/quickstart/linux-tracing

https://github.com/google/perfetto



