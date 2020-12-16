#ifndef VECTOR_H_INCLUDED
#define VECTOR_H_INCLUDED

template<typename T>
class vector{
private:
    T* p;
    int num;
public:
    vector(){
        num=0;
        p;
    }
    void add(T elem){
        T* uj=new T[num+1];
        for(int i=0; i<num; i++)
        {
            uj[i]=p[i];
        }
        num++;
        uj[num-1]=elem;
        delete[] p;
        p=uj;
    }
    void delLast()
    {
        if(num!=0)
        {
            delete &p[num-1];
        }
        num--;
    }
    void deleteAt(int idx)
    {
        if(idx>0 && idx<num)
        {
            int j=0;
            T* temp=new T[num-1];
            for(int i=0; i<num;i++)
            {
                if(i!=idx)
                {
                    temp[j]=p[i];
                    j++;
                }
            }
            num--;
            delete p;
            p=temp;
        }

    }
    T& operator[](const int idx)
    {
        if(idx>=0 && idx<num)
        {
            return p[idx];
        }
    }
    T operator[](const int idx) const
    {
        if(idx>=0 && idx<num)
        {
            return p[idx];
        }
    }
    int size()
    {
        return num;
    }
    ~vector(){
        delete[] p;

    }

};

#endif // VECTOR_H_INCLUDED
