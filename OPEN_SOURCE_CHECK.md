# Open Source Release Check

## Included

- ESP32-C3 Arduino sketch
- Arduino Nano I2C servo sketch
- Arduino Nano standalone servo test sketch
- OpenMV tracking script
- README, license, and Git ignore rules

## Excluded

- `video/` photos and video files
- Course design report document
- `ai.md` local project notes
- Local IDE metadata and build outputs
- Secret/key file patterns

## Scan Result

Run from this folder before publishing:

```powershell
rg -n -i "(password|passwd|token|secret|api[_-]?key|ssid|wifi|邮箱|电话|学号|姓名|C:\\\\Users|PRIVATE|BEGIN RSA|OPENAI|github|ghp_|sk-)" .
```

Expected intentional matches:

- `AP_SSID`
- `AP_PASSWORD`
- README network documentation
- This check document

The included Wi-Fi password is `change-me`, a placeholder for users to replace.
