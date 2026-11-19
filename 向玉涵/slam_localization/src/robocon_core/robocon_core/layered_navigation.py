"""A* routing across separate floor grids and explicit transition portals."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math

import numpy as np


@dataclass(frozen=True, order=True)
class Cell:
    floor: str
    row: int
    col: int


@dataclass(frozen=True)
class FloorGrid:
    floor: str
    free: np.ndarray
    resolution_m: float
    origin_xy_m: tuple[float, float] = (0.0, 0.0)

    def __post_init__(self) -> None:
        if self.free.ndim != 2 or self.free.dtype != np.bool_:
            raise ValueError("free must be a two-dimensional boolean array")
        if self.resolution_m <= 0:
            raise ValueError("resolution must be positive")

    def contains(self, cell: Cell) -> bool:
        return (
            cell.floor == self.floor
            and 0 <= cell.row < self.free.shape[0]
            and 0 <= cell.col < self.free.shape[1]
        )

    def is_free(self, cell: Cell) -> bool:
        return self.contains(cell) and bool(self.free[cell.row, cell.col])

    def xy(self, cell: Cell) -> tuple[float, float]:
        return (
            self.origin_xy_m[0] + (cell.col + 0.5) * self.resolution_m,
            self.origin_xy_m[1] + (cell.row + 0.5) * self.resolution_m,
        )


@dataclass(frozen=True)
class Transition:
    name: str
    entrance: Cell
    exit: Cell
    traversal_cost_m: float
    enabled: bool = True


@dataclass(frozen=True)
class RoutePoint:
    cell: Cell
    x_m: float
    y_m: float
    motion: str
    transition_name: str = ""


class LayeredAStar:
    """Plan on individual floor grids and cross floors only via transitions."""

    _MOVES = ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1))

    def __init__(self, floors: list[FloorGrid], transitions: list[Transition]) -> None:
        self.floors = {floor.floor: floor for floor in floors}
        self.transitions = transitions
        self._portals: dict[Cell, list[tuple[Cell, float, str]]] = {}
        for item in transitions:
            if not item.enabled:
                continue
            if not math.isfinite(item.traversal_cost_m) or item.traversal_cost_m < 0:
                raise ValueError(f"transition {item.name} must have a finite nonnegative cost")
            if item.entrance.floor not in self.floors or item.exit.floor not in self.floors:
                raise ValueError(f"transition {item.name} references an unknown floor")
            self._portals.setdefault(item.entrance, []).append((item.exit, item.traversal_cost_m, item.name))
            self._portals.setdefault(item.exit, []).append((item.entrance, item.traversal_cost_m, item.name))

    def plan(self, start: Cell, goal: Cell) -> list[RoutePoint]:
        if not self._grid(start).is_free(start) or not self._grid(goal).is_free(goal):
            raise ValueError("start and goal must lie in free cells")
        queue: list[tuple[float, int, Cell]] = [(0.0, 0, start)]
        counter = 0
        cost = {start: 0.0}
        parent: dict[Cell, tuple[Cell, str]] = {}
        while queue:
            _, _, current = heapq.heappop(queue)
            if current == goal:
                return self._reconstruct(parent, start, goal)
            for neighbor, move_cost, transition_name in self._neighbors(current):
                candidate = cost[current] + move_cost
                if candidate >= cost.get(neighbor, math.inf):
                    continue
                cost[neighbor] = candidate
                parent[neighbor] = (current, transition_name)
                counter += 1
                heapq.heappush(queue, (candidate + self._heuristic(neighbor, goal), counter, neighbor))
        raise RuntimeError(f"no route from {start} to {goal}")

    def _neighbors(self, cell: Cell):
        grid = self._grid(cell)
        for dr, dc in self._MOVES:
            other = Cell(cell.floor, cell.row + dr, cell.col + dc)
            if not grid.is_free(other):
                continue
            if dr and dc:
                if not grid.is_free(Cell(cell.floor, cell.row + dr, cell.col)):
                    continue
                if not grid.is_free(Cell(cell.floor, cell.row, cell.col + dc)):
                    continue
            yield other, grid.resolution_m * math.hypot(dr, dc), ""
        for other, portal_cost, name in self._portals.get(cell, []):
            if self._grid(other).is_free(other):
                yield other, portal_cost, name

    def _grid(self, cell: Cell) -> FloorGrid:
        try:
            return self.floors[cell.floor]
        except KeyError as error:
            raise ValueError(f"unknown floor: {cell.floor}") from error

    def _heuristic(self, cell: Cell, goal: Cell) -> float:
        # Portals can provide a shortcut even when start and goal share a floor.
        # Without a portal-aware lower bound, use Dijkstra to retain optimality.
        if self._portals or cell.floor != goal.floor:
            return 0.0
        x0, y0 = self._grid(cell).xy(cell)
        x1, y1 = self._grid(goal).xy(goal)
        return math.hypot(x1 - x0, y1 - y0)

    def _reconstruct(self, parent: dict[Cell, tuple[Cell, str]], start: Cell, goal: Cell) -> list[RoutePoint]:
        cells = [goal]
        transitions: list[str] = []
        while cells[-1] != start:
            previous, transition_name = parent[cells[-1]]
            transitions.append(transition_name)
            cells.append(previous)
        cells.reverse()
        transitions.reverse()
        result: list[RoutePoint] = []
        for index, cell in enumerate(cells):
            x, y = self._grid(cell).xy(cell)
            transition_name = transitions[index - 1] if index > 0 else ""
            result.append(
                RoutePoint(
                    cell=cell,
                    x_m=x,
                    y_m=y,
                    motion="traverse_level" if transition_name else "same_floor",
                    transition_name=transition_name,
                )
            )
        return result
