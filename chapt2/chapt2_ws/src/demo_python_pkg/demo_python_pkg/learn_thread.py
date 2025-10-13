import threading
import requests

class Download:
    def download(self, url, callback_word_count):
        print(f"start download, url: {url}, thread id: {threading.get_ident()}")
        response = requests.get(url)
        response.encoding = 'utf-8'
        callback_word_count(url, response.text)

        
    def start_download(self, url, callback_word_count):
        thread_download = threading.Thread(target=self.download, args=(url, callback_word_count))
        thread_download.start()


def word_count(url, result):
    print(f"{url}: length: {len(result)}")

def main():
    download = Download()
    download.start_download('http://0.0.0.0:8000/novels1.txt', word_count)
    download.start_download('http://0.0.0.0:8000/novels2.txt', word_count)
    download.start_download('http://0.0.0.0:8000/novels3.txt', word_count)

