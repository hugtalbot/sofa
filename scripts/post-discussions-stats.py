#!python


import os
from python_graphql_client import GraphqlClient
import requests
from importlib import util
import asyncio
from nio import (AsyncClient, SyncResponse, RoomMessageText)


client = GraphqlClient(endpoint="https://api.github.com/graphql")
github_token = os.environ['GITHUB_TOKEN']

# List of the repository to scan
repos=[['sofa-framework','sofa']]


def computeListOfOpenDiscussionsPerCategory():
  for repo in repos:

    owner = repo[0]
    name = repo[1]

    has_next_page = True
    after_cursor = None

    categories = []
    discussions_numbers = []


    while has_next_page:
        # Trigger the query on discussions
        data = client.execute(
            query = make_query_discussions(owner, name, after_cursor),
            headers = {"Authorization": "Bearer {}".format(github_token)},
        )

        # Process each discussion
        for discussion in data["data"]["repository"]["discussions"]["nodes"]:

            # Exit if discussion is closed or answered
            if discussion["closed"] == True or discussion["isAnswered"] == True :
                continue

            categories.append(discussion["category"]["name"])
            discussions_numbers.append(discussion["number"])

        # save if request has another page to browse and its cursor pointers
        has_next_page = data["data"]["repository"]["discussions"]["pageInfo"]["hasNextPage"]
        after_cursor = data["data"]["repository"]["discussions"]["pageInfo"]["endCursor"]
  return categories, discussions_numbers

def printDiscussionsPerCategory(categories, discussions_numbers):
    Message = "*****************\n#### GHD weekly report\n"
    categoryDone = []
    for category in categories:
        tempVecID = []
        tempMessage = ""
        if category in categoryDone:
            continue

        for i,number in enumerate(discussionsNumbers):
            if categories[i] == category:
                tempVecID.append(number)
        tempMessage = "Category "+str(category)+":"

        for id in tempVecID:
            tempMessage = tempMessage + " [#"+ str(id) +"](https://github.com/sofa-framework/sofa/discussions/"+ str(id) +"),"

        Message = Message + tempMessage + "\n"
        categoryDone.append(category)

    Message = Message + "*****************\n"


    asyncio.run(sendMessageToGitter(Message))

    return


async def sendMessageToGitter(Message):

    # async_client = AsyncClient(
    #     "https://app.gitter.im", "hugtalbot"
    # )
    #
    # token = os.getenv('GITHUB_TOKEN')
    #
    # response = await async_client.login(token)
    # print(response)
    #
    #
    # content = {
    #     "body": "Good matin!",
    #     "msgtype": "m.text"
    # }
    #
    # room_id = "!XWVzIQpwOMjHkyTsNE:gitter.im"
    #
    # await async_client.room_send(room_id, 'm.room.message', content)



    api_url = f'https://api.gitter.im/v1/rooms/!XWVzIQpwOMjHkyTsNE:gitter.im/chatMessages'
    token = os.getenv('GITHUB_TOKEN')
    headers = {
        'Content-Type': 'application/json',
        'Accept': 'application/json',
        'Authorization': 'Bearer '+str(token)
    }
    body = {
        'text': 'good morning!',
    }

    response = requests.post(api_url, headers=headers, json=body)

    if response.status_code == 200:
        print('Message sent successfully!')
    else:
        print(f'Error sending message. Status code: {response.status_code}')
        print(response.text)

    return

# Query to access all discussions
def make_query_discussions(owner, name, after_cursor=None):
    query = """
      query {
        repository(owner: "%s" name: "%s") {
          discussions(answered: false, first: 10, after:AFTER) {
            totalCount
            pageInfo {
              hasNextPage
              endCursor
            }
            nodes {
              number
              isAnswered
              closed
              category {
                name
              }
            }
          }
        }
      }""" % (owner, name)
    return query.replace("AFTER", '"{}"'.format(after_cursor) if after_cursor else "null")



#==========================================================
# STEPS computed by the script
#==========================================================
# 1 - get the discussion to be warned and closed
result = computeListOfOpenDiscussionsPerCategory()
categories = result[0]
discussionsNumbers = result[1]

printDiscussionsPerCategory(categories, discussionsNumbers)
#==========================================================