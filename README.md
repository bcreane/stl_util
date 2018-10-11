# stl_util
some C++ utility classes 

Provide a key/value hash with least recently used eviction when hash size exceeds capacity.

Example:     LruHash<int, std::string> fun(10);
             fun.insert(2, std::string("fun"));

             std::string value;
             fun.lookup(2, &value);
             fun.erase(2);
