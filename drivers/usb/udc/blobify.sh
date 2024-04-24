#!/bin/sh
# Remove anything useful from the source

set -- udc_usb23blob.c
symbols=$(sed -rn '/static / s/^[^ #].* \**([^ *]+)\(.+\)$/\1/ p' "$@")

echo Symbols stripped away:
printf '%s\n' $symbols

i=0;
for x in $symbols; do i=$((i+1))
    sed -ri "s/$x([^a-z_0-9])/fn$i\\1/g" "$@"
done
