from PIL import Image
import requests
from transformers import Blip2ForConditionalGeneration, AutoProcessor
import torch
import sys
import os
import torch
import transformers
import huggingface_hub
import requests
from PIL import Image
from time import sleep

def main(args=None):
    print("Python Version:", sys.version)
    print("PyTorch Version:", torch.__version__)
    print("Transformers Version:", transformers.__version__)
    print("Hugging Face Hub Version:", huggingface_hub.__version__)
    print("Hugging Face Transformers Version:", transformers.__version__)
    # Prüfen, ob CUDA verfügbar ist
    print("CUDA verfügbar:", torch.cuda.is_available())
    if torch.cuda.is_available():
        print("CUDA Version:", torch.version.cuda)
        print("CUDA Device Name:", torch.cuda.get_device_name(0))
    # Modell laden
    model_name = "Salesforce/blip2-opt-2.7b"
    processor = AutoProcessor.from_pretrained(model_name)
    model = Blip2ForConditionalGeneration.from_pretrained(model_name, torch_dtype=torch.float16)

    # Pfad zum geladenen Modell anzeigen
    model_path = transformers.utils.hub.cached_file(model_name, "config.json")
    print("Modell wurde geladen von:", os.path.dirname(model_path))

    device = "cuda" if torch.cuda.is_available() else "cpu"
    model.to(device)

    url = 'https://storage.googleapis.com/sfr-vision-language-research/LAVIS/assets/merlion.png' 
    image = Image.open(requests.get(url, stream=True).raw).convert('RGB')   

    prompt = "this is a picture of"

    inputs = processor(image, text=prompt, return_tensors="pt").to(device, torch.float16)

    generated_ids = model.generate(**inputs, max_new_tokens=20)
    generated_text = processor.batch_decode(generated_ids, skip_special_tokens=True)[0].strip()
    print(generated_text)

    while True:
        print("looping")
        sleep(1)




    # processor = AutoProcessor.from_pretrained("Salesforce/blip2-flan-t5-xl")
    # model = Blip2ForConditionalGeneration.from_pretrained("Salesforce/blip2-flan-t5-xl")

if __name__ == "__main__":
    main()
