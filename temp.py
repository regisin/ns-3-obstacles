# // const admin = require("firebase-admin");
# // const serviceAccount = require("./raiderslounge-firebase-adminsdk-xl6ea-467f50b5ea");
# // admin.initializeApp({
# //   credential: admin.credential.cert(serviceAccount),
# //   databaseURL: "https://raiderslounge.firebaseio.com"
# // });
# // const db = admin.firestore();
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
cred = credentials.Certificate("raiderslounge-firebase-adminsdk-xl6ea-467f50b5ea.json")
firebase_admin.initialize_app(cred)
db = firestore.client()
import google.cloud.exceptions

from os.path import join, realpath, dirname
import json
import math
import time
import requests
import pickle
import numpy as np

# seconds? NOT USED ANYMORE
UNTIL = 2838
# seconds
GET_INTERVAL = 15
# api base url, got from chrome dev tools
BASE_URL = "https://api.gymhuntr.com/api"
# looks like it's a fixed value
MONSTER = "83jhs"
# from chrome dev tools console: navigator.userAgent
USER_AGENT = "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_13_3) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/63.0.3239.132 Safari/537.36"
# also looks liked a fixed value
HASH_CHECK = "57b34b3eca72eed3178b785dcca4289g4"

def calcTimeUntil(_time, _lat, _lon, _cfid):
    return  (_lat * _cfid) + (_lon * _cfid) + _time

def toRadians(_angle):
    return _angle * (math.pi / 180)


def requestCFID(_lat, _lon):
    headers = {"User-Agent": USER_AGENT}
    qs = {"latitude": _lat, "longitude": _lon, "hashCheck": HASH_CHECK}
    uri = BASE_URL + "/authorise"
    r = requests.get(uri, headers=headers, params=qs)
    if 'cf-id' in r.headers:
        return int(r.headers['cf-id'])
    else:
        return -1

def requestGyms(_lat, _lon):

    _cfid = requestCFID(_lat, _lon)
    if _cfid == -1: return []

    _time = math.floor(time.time())
    _timeUntil = calcTimeUntil(_time, _lat, _lon, _cfid)

    uri = BASE_URL + "/gyms"
    headers = {"User-Agent": USER_AGENT}
    qs = {"latitude": _lat, "longitude": _lon, "hashCheck": HASH_CHECK, "monster": MONSTER, "timeUntil": _timeUntil, "time": _time}

    r = requests.get(uri, headers=headers, params=qs)
    return r.json()['gyms']


# This part saves to firebase firestore
# use this bit later, when requester is done, then from my local db to firebase, respecting the 20k limit per day

def saveGymToFirestore(_gymStr):
    _gym = json.loads(_gymStr)
    gymRef = db.collection(u'gyms').document(u'{}'.format(_gym['gym_id']))
    try:
        gymRef.get()
    except google.cloud.exceptions.NotFound:
        gymObj = {
            u'name': u'{}'.format(_gym['gym_name']),
            u'location': firestore.GeoPoint(_gym['location'][0], _gym['location'][1]),
            u'enabled': _gym['enabled'],
            u'url': u'{}'.format(_gym['url']),
            u'inid': u'{}'.format(_gym['gym_inid'])
        }
        gymRef.set(gymObj)
        




# gyms = requestGyms(39.5502358,-119.8158075)


MIN_LAT = -90.0
MAX_LAT = 90.0
MIN_LON = -180.0
MAX_LON = 180.0
STEP_LAT = 0.05

counter = 0

GYMS = []
for lat in np.arange(MIN_LAT, MAX_LAT, STEP_LAT):
    STEP_LON = 5.0 / (111.32 * np.cos(toRadians(lat)))
    for lon in np.arange(MIN_LON, MAX_LON, STEP_LON):
        gyms = requestGyms(lat, lon)
        for gymStr in gyms:
            gym = json.loads(gymStr)
            gymObj = {
                u'name': u'{}'.format(gym['gym_name']),
                u'location': firestore.GeoPoint(gym['location'][0], gym['location'][1]),
                u'enabled': gym['enabled'],
                u'url': u'{}'.format(gym['url']),
                u'inid': u'{}'.format(gym['gym_inid'])
            }
            counter += 1
            GYMS.append(gymObj)
            if counter == 20000:
                print("Counter:", counter, lat, lon)
                pickle.dump(GYMS, join(dirname(realpath(__file__)), "pickles", str(counter) + ".p"))
                counter = 0
                GYMS = []
