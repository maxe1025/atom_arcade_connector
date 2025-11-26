env = SConscript("godot-cpp/SConstruct")

env.Append(CPPPATH=["src/"])
sources = Glob("src/*.cpp") + Glob("src/controller/*.cpp")

library = env.SharedLibrary("connector/bin/connector{}{}".format(env["suffix"],env["SHLIBSUFFIX"]), source = sources)

Default(library)