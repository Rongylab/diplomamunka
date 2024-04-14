import sys
import importlib


module_name = "module_1" 
function_name = "kukifv"
#= importlib.import_module("module_1.py")
# module_obj = __import__(module_name)

module = importlib.import_module(module_name)
# Use getattr to dynamically get the function from the module
function = getattr(module, function_name)


function("Mukodik a dolog")





# create a global object containging our module
# globals()[module_name] = module_obj


# # args example
# class testClass():
#     def __init__(self, args):
#         print(args[0][1])


# def main(*args):
#     print("Main has started run")

#     module_obj.kukifv("egy")
#     # print(args[0][1])

#     # print("Create class")

#     # tc = testClass(args)

# if __name__ == '__main__':
#     main(sys.argv[1:])


# # kwargs example --> not working yet
# class testClass():
#     def __init__(self, kwargs):
#         print(kwargs)


# def main(**kwargs):
#     print("Main has started run")
#     print(kwargs)

#     print("Create class")

#     tc = testClass(kwargs)

# if __name__ == '__main__':
#     main(sys.argv[1:])



# # defining car class
# class car():
#     # args receives unlimited no. of arguments as an array
#     def __init__(self, **kwargs):
#         # access args index like array does
#         self.speed = kwargs['s']
#         self.color = kwargs['c']
 
 
# # creating objects of car class
# audi = car(s=200, c='red')
# bmw = car(s=250, c='black')
# mb = car(s=190, c='white')
 
# # printing the color and speed of cars
# print(audi.color)
# print(bmw.speed)









# # defining car class
# class car():
#     # args receives unlimited no. of arguments as an array
#     def __init__(self, **kwargs):
#         # access args index like array does
#         self.speed = kwargs['s']
#         self.color = kwargs['c']
 
 
# # creating objects of car class
# audi = car(s=200, c='red')
# bmw = car(s=250, c='black')
# mb = car(s=190, c='white')
 
# # printing the color and speed of cars
# print(audi.color)
# print(bmw.speed)
    
# import sys

# def main(*args):
#     print("Arguments received:", args)
#     # Here you can process the arguments as needed

# if __name__ == '__main__':
#     # Extract command-line arguments excluding the script name
#     args = sys.argv[1:]
#     main(*args)