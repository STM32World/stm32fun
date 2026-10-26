
## Generate key

##

`openssl ecparam -name prime256v1 -genkey -noout -out private_key.pem`

## Generate public key

`openssl ec -in private_key.pem -pubout -out public_key.pem`
