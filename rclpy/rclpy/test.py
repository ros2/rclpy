from __future__ import annotations

from abc import abstractmethod
from typing import Generic, TypeVar, Iterator, Protocol

_Iterator = TypeVar('_Iterator', bound=Iterator, covariant=True)

T = TypeVar('T')


class SomeContainer(Generic[T]):
    iterator: Iterator[T]

    def __init__(self, iterator: Iterator[T]):
        self.iterator = iterator

    def get_iterator(self) -> Iterator[T]:
        return self.iterator


class SomeContainerProtocol(Protocol[_Iterator]):
    @abstractmethod
    def __init__(self, iterator: _Iterator):
        pass

    @abstractmethod
    def get_iterator(self) -> _Iterator:
        pass


_SomeContainer = TypeVar('_SomeContainer', bound='SomeContainerProtocol', covariant=True)


class Thingy(Generic[_SomeContainer]):
    # container: _SomeContainer

    def __init__(self, container: _SomeContainer):
        self.container = container

    def foo(self: Thingy[SomeContainerProtocol[_Iterator]]) -> _Iterator:
        return self.container.get_iterator()

    def bar(self) -> _SomeContainer:
        return self.container


thingy = Thingy(SomeContainer((range(10).__iter__())))

reveal_type(thingy)
reveal_type(thingy.foo)
reveal_type(thingy.foo())
reveal_type(thingy.bar())