#include <v8.h>
#include <node.h>
#include <iostream>
#include <cstdlib>
#include <sstream>
#include <stdio.h>

using namespace node;
using namespace v8;

extern void startControl();
extern void updateInput(unsigned int, unsigned int, unsigned int, unsigned int);

Handle<Value> start(const Arguments& args) {
	HandleScope scope;

	startControl();

	return scope.Close(String::New("Start Control"));
}

Handle<Value> update(const Arguments& args) {
	HandleScope scope;

	if (args.Length() < 4) {
        ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
        return scope.Close(Undefined());
	}

	if (!args[0]->IsNumber() || !args[1]->IsNumber()
			|| !args[2]->IsNumber() || !args[3]->IsNumber()) {
        ThrowException(Exception::TypeError(String::New("Arguments are not number")));
        return scope.Close(Undefined());
	}

	updateInput(args[0]->ToNumber()->IntegerValue(), args[1]->ToNumber()->IntegerValue(),
			args[2]->ToNumber()->IntegerValue(), args[3]->ToNumber()->IntegerValue());

	return scope.Close(String::New("Update Input"));
}

void init(Handle<Object> target) {
	target->Set(String::NewSymbol("start"),
		FunctionTemplate::New(start)->GetFunction());
	target->Set(String::NewSymbol("update"),
		FunctionTemplate::New(update)->GetFunction());
}
NODE_MODULE(controls, init);
