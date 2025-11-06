from demo_python_pkg.person_node import PersonNode

class WriterNode(PersonNode):
    def __init__(self, node_name: str, name: str, age: int, book: str) -> None:
        super().__init__(node_name, name, age)
        print('WriterNode 的 __init__ 方法被调用')
        self.book = book

def main():
    node = WriterNode('writer_node', 'larry', 23, 'Python 从入门到放弃')
    node.eat('fishros')  # 调用 PersonNode 的 eat 方法