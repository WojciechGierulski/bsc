import pickle

with open("1.pickle", "rb") as file:
    a = pickle.load(file, encoding='latin1')
    print a.dtype

print(a)