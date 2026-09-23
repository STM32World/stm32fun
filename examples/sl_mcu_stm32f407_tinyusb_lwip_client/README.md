Test:
```
for note in 3C 3E 40 41 43 45 47 48 47 45 43 41 40 3e 3c; do amidi -p hw:1 -S "90 $note 64"; sleep 0.4; amidi -p hw:1 -S "80 $note 00"; done
```
