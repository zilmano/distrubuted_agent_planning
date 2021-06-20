#include <iostream>
class B;
void B::print(int num);
//void B::

class A {
public:
    void action() {
    	b_->print(5);
    };

private:
	B* b_;
};


class B {
	public:
		B(): id_(0) {};

		void print(int num) {
			std::cout << id_ << ": " << num << std::endl;
		};

	private:
		int id_;
};

int main() {

	return 0;
}


