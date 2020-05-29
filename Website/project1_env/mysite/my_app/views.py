from django.shortcuts import render

# Create your views here.

def home_screen_view(request):
    print(request.headers)
    context = {}
    #context["some_string"] = "this is some string i'm passing to the views"
    #context["some_number"] = 1234512312412
    #context = {"some_string" : "this is some string i'm passing to the views",
        #"some_number" : 1234512312412}
    list_of_values =[]
    list_of_values.append("first_values")
    list_of_values.append("second_values")
    list_of_values.append("third_values")
    
    context["list_values"] = list_of_values
    print(context)
    return render(request, "my_app/home.html", context)