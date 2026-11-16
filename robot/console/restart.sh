#!/usr/bin/env bash
# Restart the console web service (systemd user unit fmp-console). Log: ~/fmp_logs/console.log
systemctl --user restart fmp-console && sleep 2 && tail -2 ~/fmp_logs/console.log
