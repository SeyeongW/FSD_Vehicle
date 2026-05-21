# Package Integration

Waver is added as `src/waver_patrol` and upstream Waveshare packages are left unchanged.

The safe command path is:

manual/Nav2/patrol intent -> command mux -> sanitizer -> acceleration limiter -> runaway guard ->
serial JSON bridge -> ESP32 `{"T":1,"L":left,"R":right}`.

The existing Waveshare `ugv_driver` uses a different command JSON (`T:13`, `X`, `Z`). Do not run
`ugv_driver` and Waver serial bridge as simultaneous base drivers unless they target different hardware
and you understand the command path.
