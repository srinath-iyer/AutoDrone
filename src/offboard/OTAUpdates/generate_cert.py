# generate_cert.py
from cryptography import x509
from cryptography.x509.oid import NameOID
from cryptography.hazmat.primitives import serialization, hashes
from cryptography.hazmat.primitives.asymmetric import rsa
from datetime import datetime, timedelta, timezone
from pathlib import Path

# Read hostname from a file
hostname_path = Path("src/offboard/OTAUpdates/hostname.txt")
hostname = hostname_path.read_text().strip()

# Generate private key
key = rsa.generate_private_key(public_exponent=65537, key_size=2048)

# Build subject/issuer name
subject = issuer = x509.Name([
    x509.NameAttribute(NameOID.COUNTRY_NAME, u"US"),
    x509.NameAttribute(NameOID.STATE_OR_PROVINCE_NAME, u"North Carolina"),
    x509.NameAttribute(NameOID.LOCALITY_NAME, u"Durham"),
    x509.NameAttribute(NameOID.ORGANIZATION_NAME, u"AutoDrone"),
    x509.NameAttribute(NameOID.COMMON_NAME, hostname),
])

# Build certificate with timezone-aware datetime
cert = (
    x509.CertificateBuilder()
    .subject_name(subject)
    .issuer_name(issuer)
    .public_key(key.public_key())
    .serial_number(x509.random_serial_number())
    .not_valid_before(datetime.now(timezone.utc))
    .not_valid_after(datetime.now(timezone.utc) + timedelta(days=365))
    .sign(key, hashes.SHA256())
)

# Output files
certs_dir = Path("src/offboard/OTAUpdates/certs")
certs_dir.mkdir(parents=True, exist_ok=True)

(cert_path := certs_dir / "cert.pem").write_bytes(cert.public_bytes(serialization.Encoding.PEM))
(key_path := certs_dir / "key.pem").write_bytes(
    key.private_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PrivateFormat.TraditionalOpenSSL,
        encryption_algorithm=serialization.NoEncryption(),
    )
)

print(f"✅ Certificate saved to {cert_path}")
print(f"✅ Key saved to {key_path}")
