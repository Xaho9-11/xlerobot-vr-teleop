#!/usr/bin/env bash
# Restart the robot core service (systemd user unit fmp-core; it folds and releases on stop). Log: ~/fmp_logs/core.log
systemctl --user restart fmp-core && sleep 4 && grep -v "^{\|motor list\|expected model\|^$\|  - " ~/fmp_logs/core.log | tail -3
