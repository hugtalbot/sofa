import os
import sys
from urllib.request import urlopen
from bs4 import BeautifulSoup


if "DISCORD_WEBHOOK_URL" in os.environ:
    discord_token = os.environ['DISCORD_WEBHOOK_URL']
else:
    print("DISCORD_WEBHOOK_URL environment variable is missing.")
    sys.exit(1)

# Function posting a message on Discord
def postOnDiscord(message):
    payload = {
        'content' : message,
        'username' : 'Github activity tracking',
        'flags' : 4,
    }
    response = requests.post(discord_token, json=payload)
    print("Status: "+str(response.status_code)+"\nReason: "+str(response.reason)+"\nText: "+str(response.text))
    return


url = "https://www.sofa-framework.org/update-github-releases.php"
html = urlopen(url).read()
soup = BeautifulSoup(html, features="html.parser")

for script in soup(["script", "style"]):
    script.extract()

text = soup.get_text()

for line in text.splitlines():
    postOnDiscord(line)

postOnDiscord("=========================\n\n")
