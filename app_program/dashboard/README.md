# G7 dashboard

Live MQTT telemetry, barcode results, robot log and mapping/START/STOP for the
robot. Node.js 18+, no npm dependencies.

```sh
bash run_live_dashboard.sh     # from the project root: real robot, commands enabled
npm run lan                    # here: real robot, read-only
npm test                       # here: host tests
```

See [docs/DASHBOARD.md](../../docs/DASHBOARD.md) for all modes, views, the
mapping-then-START flow and environment variables.
