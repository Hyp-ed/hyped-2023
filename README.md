# HYPED 2023

This code base is meant to be used with the yet-to-be-named pod prototype that is due to be built in the academic year 2022/23. There still are a lot of question marks involved so the full functionality is not entirely clear.

However, even if the new pod were to be identical with Greyfriars Poddy (2022) this rewrite would be beneficial nonetheless. 
There are many aspects of the old code base that we can only change by restarting the whole project from 'scratch'.

## Contributing

All the C++ code should follow the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).

Further, we aim to maintain the following rules which are based on the [NASA's Ten Rules for Safety Critical Coding](https://pixelscommander.com/wp-content/uploads/2014/12/P10.pdf).

1. All loops must have a fixed upper-bound. It must be trivially possible for a 
checking tool to prove statically that a preset upper-bound on the number of iterations 
of a loop cannot be exceeded. If the loop-bound cannot be proven statically, the rule 
is considered violated.
2. Do not use dynamic memory allocation after initialisation.
3. No function should be longer than 60 lines of code.
4. The assertion density of the code should average to a minimum of two 
assertions per function. Assertions are used to check for anomalous conditions that 
should never happen in real-life executions. Assertions must always be side-effect 
free and should be defined as Boolean tests. When an assertion fails, an explicit 
recovery action must be taken, e.g., by returning an error condition to the caller of the 
function that executes the failing assertion. Any assertion for which a static checking 
tool can prove that it can never fail or never hold violates this rule. (I.e., it is not 
possible to satisfy the rule by adding unhelpful “assert(true)” statements.) 
5. Data objects must be declared at the smallest possible level of scope.
6. The return value of non-void functions must be checked by each calling 
function, and the validity of parameters must be checked inside each function.
7. The use of the preprocessor must be limited to the inclusion of header files and compile time arguments. (e.g. operating system and architecture)
8. The use of pointers should be restricted. In particular, the keywords `new` and `delete` shall never be used and raw pointers shall only be viable as an alternative to `std::optional<T&>`, which is ambiguous and therefore an illegal type.
9. All code must be compiled, from the first day of development, with all 
compiler warnings enabled at the compiler’s most pedantic setting. All code must 
compile with these setting without any warnings. All code must be checked daily with 
at least one, but preferably more than one, state-of-the-art static source code analyzer 
and should pass the analyses with zero warnings.

## Programming Language

We have the unique opportunity to change programming languages so a lot of thought has been put into this. HYPED has been using C++ since its inception so we have been able to observe the language's shortcomings. The three primary drawbacks are

1. C++ looks familiar to new programmers. This is a false sense of security. It turns out that C++, or more specifically the modern C++ that we aim to write, has practically nothing in common with C and Java even though they are syntactically very similar and some pieces of code may be copied from one of the languages to another. This has lead to C++ being treated like its cousins which usually results in bad practices and unmaintainable code.
2. C++ is difficult to set up. Getting the build system, the includes and the linking correct is non-trivial. Further, the procedure is entirely non-standard. We therefore risk having to rewrite the entire build system every couple of years because the last person who understood it is no longer around.
3. The error messages are inconsistent and mostly unhelpful to newcomers. Especially when templates and macros are involved, one may end up with a compiler output that is several kilobytes in size.

There are, however, many things that C++ does well - especially when compared to the alternatives. First and foremost we would like a predictable cost model. This rules out almost all other mainstream languages, e.g. Python, Java, Haskell, due to their grabage collectors. Switching to C would be step back so the only real contender is Rust.

There are three main problems the Rust programming language has:

1. The barrier of entry is high, arguably higher than C++'s. It is no longer possible to be oblivious to the internals of the language because the compiler is a lot stricter. While this is the main selling point for Rust within the community, it may be daunting to newcomers if they can't write simple programs anymore.
2. Traits, one of Rust's core features, are prone to abuse. While abstraction is a powerful tool, especially when developing libraries or other shared code, HYPED is aiming to avoid it as much as possible. In most cases it will be more of a code obfuscation tool than anything else.
3. Rust is not the most popular language. This harms our recruitment, the support we may hope to find online and our career prospects. While being a good Rust programmer will bring you closer to being a good programmer in general than most other programming languages ever will, it is understandable that many members would prefer using a language that is directly applicable to the work they may carry out in the future.
