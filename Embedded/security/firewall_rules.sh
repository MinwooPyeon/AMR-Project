# Clear existing rules
iptables -F
iptables -X
iptables -t nat -F
iptables -t nat -X

# Set default policies
iptables -P INPUT DROP
iptables -P FORWARD DROP
iptables -P OUTPUT ACCEPT

# Allow loopback
iptables -A INPUT -i lo -j ACCEPT
iptables -A OUTPUT -o lo -j ACCEPT

# Allow established connections
iptables -A INPUT -m state --state ESTABLISHED,RELATED -j ACCEPT

# Allow SSH (port 22)
iptables -A INPUT -p tcp --dport 22 -j ACCEPT

# Allow MQTT (port 1883)
iptables -A INPUT -p tcp --dport 1883 -j ACCEPT

# Allow MQTT over WebSocket (port 9001)
iptables -A INPUT -p tcp --dport 9001 -j ACCEPT

# Allow HTTP (port 8000)
iptables -A INPUT -p tcp --dport 8000 -j ACCEPT

# Allow WebSocket (port 8080)
iptables -A INPUT -p tcp --dport 8080 -j ACCEPT

# Allow HTTPS (port 443)
iptables -A INPUT -p tcp --dport 443 -j ACCEPT

# Allow ICMP (ping)
iptables -A INPUT -p icmp --icmp-type echo-request -j ACCEPT

# Rate limiting for SSH
iptables -A INPUT -p tcp --dport 22 -m state --state NEW -m recent --set --name SSH
iptables -A INPUT -p tcp --dport 22 -m state --state NEW -m recent --update --seconds 60 --hitcount 4 --name SSH -j DROP

# Rate limiting for MQTT
iptables -A INPUT -p tcp --dport 1883 -m state --state NEW -m recent --set --name MQTT
iptables -A INPUT -p tcp --dport 1883 -m state --state NEW -m recent --update --seconds 60 --hitcount 100 --name MQTT -j DROP

# Block suspicious ports
iptables -A INPUT -p tcp --dport 23 -j DROP  # Telnet
iptables -A INPUT -p tcp --dport 21 -j DROP  # FTP
iptables -A INPUT -p tcp --dport 25 -j DROP  # SMTP
iptables -A INPUT -p tcp --dport 110 -j DROP # POP3
iptables -A INPUT -p tcp --dport 143 -j DROP # IMAP

# Block common attack ports
iptables -A INPUT -p tcp --dport 135 -j DROP  # Windows RPC
iptables -A INPUT -p tcp --dport 139 -j DROP  # NetBIOS
iptables -A INPUT -p tcp --dport 445 -j DROP  # SMB
iptables -A INPUT -p tcp --dport 3389 -j DROP # RDP

# Log dropped packets
iptables -A INPUT -j LOG --log-prefix "AMR_FIREWALL_DROP: " --log-level 4

# Save rules
iptables-save > /etc/iptables/rules.v4

echo "Firewall rules configured successfully"
