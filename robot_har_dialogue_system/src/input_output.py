#!/usr/bin/env python3
import pyttsx3
import speech_recognition as sr

from log import Log

class InputOutput(object):
    def __init__(self):
        self.id = 'input_outpt'
        self.logger = Log(self.id)

        self.tts_engine = pyttsx3.init()

        self.logger.log_great('Ready.')

    def say(self, text):
        self.tts_engine.say(text)
        self.tts_engine.runAndWait()

    def listen(self):
        r = sr.Recognizer()
        with sr.Microphone() as source:
            audio = r.listen(source)

        GOOGLE_CLOUD_SPEECH_CREDENTIALS = r"""{
            "type": "service_account",
            "project_id": "virtual-ralt",
            "private_key_id": "4668a174c618f5419c004837dd9a174e26f2cdd9",
            "private_key": "-----BEGIN PRIVATE KEY-----\nMIIEvgIBADANBgkqhkiG9w0BAQEFAASCBKgwggSkAgEAAoIBAQC22PJKvbYQGg71\nfo9oTGoK9nBoJJcUXKIQgXowTU8+SyQAS34iAlXomrXMJ+8J2cHh3Hf3cUEpkljo\nPHn02mqTkHWYbR35T5SUyegdzsXtOj8oD35YXMm9BFLOXjAN6WqWy6Qj3k35sy39\nhqh6oA9qK40+S6ksxXq3706cYAnVwylqApdsVPKW3JJAzAXkdlJshavvg8V9A1ME\nCZYUkidT5JeA2Cv0x9xldwj3TyDkAEGXuTQ4ixFPXp1zKoYUqJ1SUVVioNnwg6TU\nO7z+zAQIHfycNaUh7AK9v+4F0q5aRmsMSNiGS2Zzsijpfn2k02IwCABFixWFmEfs\nw/WmjBwXAgMBAAECggEAVpSO0j5XrfMw3yUhEdGRKwS7gEzHLzCX2vn8Wk/ZNJWP\nBAa7QK5bcIuFhxjnswHmMSVctxHtOZqfN8SfOQaNjbdLP7R3i3yXzgZ3P1oQVb8N\nddI0UPIK4tJfxXFLDuNWgfYGkKdsw86I+fIf6ATPl8XsmTFpd5ZBAES0NA6zyYBS\n2qGuFe59dCrZCuKena1FuIc3yFKD4mjRLacb3vXu9sY1R/iNMDp7d6/b2b4/6daH\nOFFOWx21DGJvR6Pb02dsJ379YrqNtTWdv3K/Oy5Pov8/GTmJSxam4E1WNd/UXNbk\nDwvFZ0z8jEwEitAP3TbDLuL3Kt2T2ZVYfVPvwX4JWQKBgQD/7LxJ7AalIEx0ii+N\nfw7NNttjKvixRAGo1+M3Tb/EZ83WpeA3pO3BzujekqRcMbHwK3EIwIDSyBt6hECw\nTNt4XQnAxIWoerla/bP0yUrKSDFKzDQbR5S6YCrEBETMRqrBSpwJayGEIL+zD7fV\nPSo9OuwovCmXrcvrZ9g9Nb3MPwKBgQC25rXKs5/26cBNpQa3+M9bSamKC4f6yUZW\nFh8fd0dIAK6f59wS+hCcOe2eiTHUo0WYBMSdbEBGzUEq6Oo6Klrl0VkcjLeKcLXC\nlpsQR9a5Wm7YRpHTIrN+tHBjWaMVCCbSxvwk4Potz+RopyxYz/HVvs5FsUbwZauN\niSt6wZgaKQKBgBKMiAEbmEezlZcExPmOcfYxuajwXmIKucwRCajie7myhFrAXwXp\ncEoRwxMOsdb0yx7LJ0ApI60jT8qLd1zP1UBeDezo0X8qlfCgXsCpLkCHBvuS3Vv0\nzjTL742ReaMMpCeqTrDoEN/Qt9Q0mXBdJIiXkrJJIQKbWCJRvnyBPqZlAoGBAKhY\n3oc+okRAjzJobTC0oRdjS9u2FWkuDcF7BxaNUVzi09h3L0eAR/2kDCTaHrWbupjb\n+qTzWI5SqmL2k3EKPzQ9ZwoyM1YHvCTpPrzTgr6EkHYdLmfqDtDXAINRpBvdru5f\nHQJM2bmFGrWA+f6dF/kncPEQ5TysOCVEVnLc8kVRAoGBAL/tvfwYGit7BZItGvxO\n9gVuDO+1n0mN1x7tTd19DdmGNZvRprlsCLgjg81Rn/74sv/ERcmvlkCG6XmOBFGw\ndSgq3Sq/u4IQ7nYGSjHALgoQb+DPQf+v1qUqqEqTM2yGryMCmg2gwZ8FmtxXqLz/\nwyqsyM8hcV8SOb8dUZtK38oF\n-----END PRIVATE KEY-----\n",
            "client_email": "stt-413@virtual-ralt.iam.gserviceaccount.com",
            "client_id": "109588610473603576428",
            "auth_uri": "https://accounts.google.com/o/oauth2/auth",
            "token_uri": "https://oauth2.googleapis.com/token",
            "auth_provider_x509_cert_url": "https://www.googleapis.com/oauth2/v1/certs",
            "client_x509_cert_url": "https://www.googleapis.com/robot/v1/metadata/x509/stt-413%40virtual-ralt.iam.gserviceaccount.com"
            }
            """
        try:
            result = r.recognize_google_cloud(audio, credentials_json=GOOGLE_CLOUD_SPEECH_CREDENTIALS)
            print("Google Cloud Speech thinks you said", result)
            return result
        except sr.UnknownValueError:
            print("Google Cloud Speech could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Cloud Speech service; {0}".format(e))

if __name__ == '__main__':
    io = InputOutput()
    io.listen()