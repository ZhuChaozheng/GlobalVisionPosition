class MyData
{
public:
    MyData() : A(0), X(0), id() {}
    void read(const FileNode& node)                          //Read serialization for this class
    {
        A = (int)node["A"];
        X = (float)node["X"];
        id = (string)node["id"];
    }
public:   // Data Members
   int A;
   float X;
   string id;
};