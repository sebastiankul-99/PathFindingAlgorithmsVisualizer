import pygame
import numpy as np
from tkinter import messagebox, Tk, OptionMenu, StringVar, Message, Label
from queue import PriorityQueue, Queue
from collections import defaultdict
from tkinter import *
from collections import deque

RESTART_OPTIONS = [
"Yes",
"No"
]
OPTIONS = [
"A*",
"dijkstra",
"DFS",
"BFS"
]
pygame.init()
WIDTH = 800
HEIGHT = 800
COLLUMNS = 40


def path_unreachable():
	master = Tk()
	master.title("Path not found")
	tk_win_width = 250
	tk_win_height = 80
	screen_width = master.winfo_screenwidth()
	screen_height = master.winfo_screenheight()
	center_window = str(tk_win_width) + "x" + str(tk_win_height) + "+" + str(
		screen_width // 2 - tk_win_width // 2) + "+" + str(screen_height // 2 - tk_win_height // 2)
	master.geometry(center_window)

	mes1 = Message(master, text="Path not reachable", width=200)
	mes1.pack()

	btn = Button(master, text='Ok', bd='5', command=master.destroy)
	btn.pack()
	btn.place(x=95, y=30, width=60)
	master.mainloop()
	return


class Point:
	def __init__(self, i,j,grid):
		self.x =i
		self.y =j
		self.neighbours = []
		self.cost = grid.arr[i,j]
		self.index = (i * grid.rows) + j
		self.parent = None
		self.distance = None
		if i>1:
			self.neighbours.append(((i-1) * grid.rows) + j)
		if j>1:
			self.neighbours.append((i * grid.rows) + j-1)
		if i < grid.rows-2:
			self.neighbours.append(((i+1) * grid.rows) + j)
		if j < grid.columns-2:
			self.neighbours.append((i * grid.rows) + j+1)

	def get_index(self,i,j,grid):
		self.index = (i * grid.rows) + j

	def __lt__(self, other):
		return self.cost > other.cost


class Grid:
	def __init__(self, numOfRows, numOfColumns):
		self.rows = numOfRows
		self.columns = numOfColumns
		self.arr = np.ones((numOfRows,numOfRows))
		self.cellWidth = WIDTH//self.rows
		for i in range(numOfRows):
			self.arr[i, 0]= 9999999
			self.arr[i, self.rows-1] = 9999999
			self.arr[0, i] = 9999999
			self.arr[self.rows-1, i] = 9999999

	def getCords(self, tupl, button):
		x, y = tupl
		x = x//self.cellWidth
		y = y // self.cellWidth
		if button ==0:
			if self.arr[x,y] == 1:
				self.arr[x,y] = 9999999
		elif button == 1:
			self.arr[x, y] = 1

		elif button==2:
			self.arr[x, y] = 2

	def getCordsFromIndex(self, index):
		return (index // self.rows,index % self.rows )

	def removePoint(self, cords):
		x, y = cords
		self.arr[x,y] = 1

	def checkStartingPoint(self,cords):
		x, y = cords
		self.arr[x, y] = 2

	def draw(self, win):
		for i in range(self.rows):

			for j in range(self.rows):
				if self.arr[i,j] == 9999999:
					pygame.draw.rect(win,(0,0,0),(i*self.cellWidth,j*self.cellWidth,self.cellWidth,self.cellWidth))
				elif  self.arr[i,j] == 2:
					pygame.draw.circle(win, (255, 128, 128),
					                   (i * self.cellWidth + self.cellWidth // 2,
					                    j * self.cellWidth + self.cellWidth // 2), self.cellWidth // 2)
				elif self.arr[i, j] == 3:
					pygame.draw.rect(win, (53, 243, 235),
					                 (i * self.cellWidth, j * self.cellWidth, self.cellWidth, self.cellWidth))
				elif self.arr[i, j] == 4:
					pygame.draw.rect(win, (255, 204, 102),
					                 (i * self.cellWidth, j * self.cellWidth, self.cellWidth, self.cellWidth))

		for i in range(0,HEIGHT+1,int(self.cellWidth)):
			pygame.draw.line(win,(0,0,0),(i,0),(i,HEIGHT))
			pygame.draw.line(win, (0, 0, 0), (0, i),(WIDTH,i))

	def reset(self):
		for i in range(self.rows):
			for j in range(self.rows):
				self.arr[i,j]=1
		for i in range(self.rows):
			self.arr[i, 0]= 9999999
			self.arr[i, self.rows-1] = 9999999
			self.arr[0, i] = 9999999
			self.arr[self.rows-1, i] = 9999999


def getCordsIndex(tupl):
	x, y = tupl
	x = x//(WIDTH//COLLUMNS)
	y = y//(WIDTH//COLLUMNS)
	index = (x * 40) + y
	return index


def draw(WIN, grid):
	grid.draw(WIN)
	pygame.display.update()


def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


def Astar(WIN, grid, graph, start, end):
	count = 0
	open_set = PriorityQueue()
	open_set.put((0, count, start))
	came_from = {}
	infinity = 9999999
	weight_score = {point: infinity for point in graph}
	weight_score[start] = 0
	distance_score = {point: infinity for point in graph}
	distance_score[start] = heuristic((start.x, start.y),(end.x,end.y))

	open_set_hash = {start}
	clock = pygame.time.Clock()
	while not open_set.empty():
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
		clock.tick(120)
		current = open_set.get()[2]
		open_set_hash.remove(current)

		if current.x == end.x and current.y == end.y:
			path = []
			grid.arr[end.x, end.y] = 2
			while current in came_from:
				current = came_from[current]
				path.append(current)
			path.reverse()
			for current in path:
				if grid.arr[current.x, current.y] != 2:
					grid.arr[current.x, current.y] = 4
				draw(WIN, grid)

			return

		for neighbor in current.neighbours:
			temp_weight_score = weight_score[current] + 1

			if temp_weight_score < weight_score[graph[neighbor]] and graph[neighbor].cost!= infinity:
				came_from[graph[neighbor]] = current
				weight_score[graph[neighbor]] = temp_weight_score
				distance_score[graph[neighbor]] = temp_weight_score + heuristic((graph[neighbor].x, graph[neighbor].y), (end.x, end.y))
				if graph[neighbor] not in open_set_hash:
					count += 1
					open_set.put((distance_score[graph[neighbor]], count, graph[neighbor]))
					open_set_hash.add(graph[neighbor])

					grid.arr[graph[neighbor].x, graph[neighbor].y] = 3
	

		if current.x != start.x and current.y != start.y:

			grid.arr[current.x,current.y] = 3
		draw(WIN, grid)

	path_unreachable()
	return


def dijkstra(WIN, grid, graph, start, goal):
	predecessor = {}
	grid.arr[start.x, start.y] = 2
	grid.arr[goal.x, goal.y] = 2
	unseenNodes = graph.copy()
	infinity = 9999999
	path = []
	shortest_distance = {node:infinity for node in unseenNodes}
	shortest_distance[start] = 0
	clock = pygame.time.Clock()
	run = True
	while unseenNodes and run:
		clock.tick(120)
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False
				pygame.quit()

		minNode = None
		minNodeIndex = 0

		for node in unseenNodes:
			if minNode is None:
				minNode = node
				minNodeIndex = node.index
			elif shortest_distance[node] < shortest_distance[minNode]:
				minNode = node
				minNodeIndex = node.index

		for childNode in graph[int(minNodeIndex)].neighbours:
				if graph[childNode].cost != infinity and shortest_distance[minNode] < shortest_distance[graph[childNode]]:
					shortest_distance[graph[childNode]] = graph[minNodeIndex].cost + shortest_distance[minNode]

					if grid.arr[graph[childNode].x, graph[childNode].y] == 1:
						grid.arr[graph[childNode].x, graph[childNode].y] = 3
					predecessor[graph[childNode]] = minNode
				if graph[childNode] == goal:
					run = False
					break
		draw(WIN, grid)
		unseenNodes.remove(minNode)
	currentNode = goal
	while currentNode != start:
		try:
			path.insert(0, currentNode)
			currentNode = predecessor[currentNode]
		except KeyError:
			path_unreachable()
			break
	path.insert(0, start)
	if shortest_distance[goal] != infinity:
		for i in path:
			if grid.arr[i.x, i.y]!=2:
				grid.arr[i.x, i.y] = 4
			draw(WIN, grid)


def DFS(win, grid, graph, start, end):
	infinity = 9999999
	where_to_go_next = [ ]
	where_to_go_next.append(start)
	already_visited = []
	current_node = start
	run = True
	found = False
	clock = pygame.time.Clock()
	while len(where_to_go_next)!=0 and run:
		clock.tick(120)
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
		current_node = where_to_go_next.pop()
		already_visited.append(current_node)
		if grid.arr[current_node.x, current_node.y] == 1:
			grid.arr[current_node.x, current_node.y] = 3
		if current_node.x == end.x and current_node.y == end.y:
			run = False
			found = True
		for neighbour in  current_node.neighbours:
			if graph[neighbour] not in already_visited and graph[neighbour].cost != infinity :
				where_to_go_next.append(graph[neighbour])

		draw(win,grid)
	if found:
		for i in already_visited:
			if grid.arr[i.x, i.y] == 3:
				grid.arr[i.x, i.y] = 4
			draw(win, grid)
	else:
		path_unreachable()


def BFS(win, grid, graph, start, end):
	infinity = 9999999
	already_visited, queue = list(), deque([start])
	already_visited.append(start)
	found = False
	run = True
	path = {}
	finish_path = []
	clock = pygame.time.Clock()
	while queue and run:
		clock.tick(120)
		current = queue.popleft()
		if current.x == end.x and current.y == end.y:
			found = True
			run = False
			temp = current
			found_path = False
			while not found_path:

				temp = path[temp]
				finish_path.append(temp)
				if temp.x == start.x and temp.y == start.y:
					found_path = True
					break
				finish_path.append(path[temp])
		if grid.arr[current.x, current.y] == infinity:
			continue
		if grid.arr[current.x, current.y] == 1:
			grid.arr[current.x, current.y] = 3

		draw(win, grid)
		for neighbour in current.neighbours:
			if graph[neighbour] not in already_visited and graph[neighbour] != infinity:
				already_visited.append(graph[neighbour])
				path[graph[neighbour]] = current
				queue.append(graph[neighbour])

	finish_path.reverse()
	if found:
		for i in finish_path:
			if grid.arr[i.x, i.y] == 3:
				grid.arr[i.x, i.y] = 4
			draw(win, grid)
	else:
		path_unreachable()


def chooseAlgo():
	master = Tk()
	master.title("Chose a path finding algorithm")
	tk_win_width = 350
	tk_win_height = 100
	screen_width = master.winfo_screenwidth()
	screen_height = master.winfo_screenheight()
	center_window = str(tk_win_width) + "x" + str(tk_win_height) + "+" + str(
		screen_width // 2 - tk_win_width // 2) + "+" + str(screen_height // 2 - tk_win_height // 2)
	master.geometry(center_window)

	mes1 = Message(master, text="Path finding algorithm visualizer", width=250)
	mes1.pack()
	mes1.place(x=10, y=1)
	l3 = Message(master, text='Select algorithm from a list', width=200)
	l3.pack()
	l3.place(x=10, y=30)

	variable = StringVar(master)
	variable.set(OPTIONS[0])  # default value
	w = OptionMenu(master, variable, *OPTIONS, )
	w.pack()
	w.place(x=180, y=27, width=90, )
	btn = Button(master, text='Start', bd='5', command=master.destroy)
	btn.pack()
	btn.place(x=110, y=60, width=60)
	master.mainloop()
	chosen = variable.get()
	return chosen


def shouldReset():
	Restart = Tk()
	tk_win_width = 350
	tk_win_height = 100
	screen_width = Restart.winfo_screenwidth()
	screen_height = Restart.winfo_screenheight()
	center_window = str(tk_win_width) + "x" + str(tk_win_height) + "+" + str(
		screen_width // 2 - tk_win_width // 2) + "+" + str(screen_height // 2 - tk_win_height // 2)
	Restart.title("Run again")
	Restart.geometry(center_window)

	mes = Message(Restart, text="Do you want to visualize algortihm again?", width=250)
	mes.pack()

	variable2 = StringVar(Restart)
	variable2.set(RESTART_OPTIONS[0])  # default value
	w2 = OptionMenu(Restart, variable2, *RESTART_OPTIONS, )
	w2.pack()
	w2.place(x=60, y=32, width=90, )
	btn_submit = Button(Restart, text='Submit', bd='5', command=Restart.destroy)

	btn_submit.pack()
	btn_submit.place(x=160, y=32, width=90)
	Restart.mainloop()
	chosen = variable2.get()
	return chosen


def main():
	global OPTIONS, COLLUMNS
	chosen = chooseAlgo()
	finished = False
	print(chosen)

	WIN = pygame.display.set_mode((WIDTH, HEIGHT))

	grid = Grid(COLLUMNS,COLLUMNS)
	startEndIndex= []
	points = []
	run = True
	clock = pygame.time.Clock()
	should_start = 0
	while run:
		#clock.tick(60)
		for event in pygame.event.get():

			if event.type == pygame.QUIT:
				run = False
				pygame.quit()

			if pygame.mouse.get_pressed()[0]:
				cord = pygame.mouse.get_pos()
				grid.getCords(cord,0)

			if pygame.mouse.get_pressed()[1]:
				cord = pygame.mouse.get_pos()
				grid.getCords(cord,1)
				if len(startEndIndex)>0:
					for i in startEndIndex:
						if i == getCordsIndex(cord):
							startEndIndex.remove(i)

			if pygame.mouse.get_pressed()[2]:
				cord = pygame.mouse.get_pos()

				if len(startEndIndex)<2:
					startEndIndex.append(getCordsIndex(cord))
					grid.getCords(cord,2)
				else:
					grid.removePoint(grid.getCordsFromIndex(startEndIndex[1]))
					startEndIndex.pop()
					startEndIndex.append(getCordsIndex(cord))
					grid.getCords(cord, 2)
					grid.checkStartingPoint(grid.getCordsFromIndex(startEndIndex[0]))

			if len(startEndIndex)==2:
				should_start = 2
			else:
				should_start=0

			if event.type == pygame.KEYDOWN and should_start==2:

				for i in range(grid.rows):
					for j in range(grid.columns):
						points.append(Point(i, j, grid))

				if event.key == pygame.K_SPACE:
					if chosen =="A*":
						Astar(WIN, grid, points, points[startEndIndex[0]],points[startEndIndex[1]])
					elif chosen =="dijkstra":
						dijkstra(WIN, grid, points, points[startEndIndex[0]],points[startEndIndex[1]])
					elif chosen =='DFS':
						DFS(WIN, grid, points, points[startEndIndex[0]],points[startEndIndex[1]])
					elif chosen =='BFS':
						BFS(WIN, grid, points, points[startEndIndex[0]],points[startEndIndex[1]])

					finished = True

		WIN.fill((19, 48, 57))
		draw(WIN, grid)

		if finished:
			should_restart= None
			should_restart = shouldReset()

			if should_restart =="Yes":
				should_restart = None
				grid.reset()
				points.clear()
				startEndIndex.clear()
				chosen = chooseAlgo()
				should_start = 0
				finished = False

			elif should_restart =="No":
				should_restart = None
				run = False
				finished = False
				pygame.quit()


if __name__ == '__main__':
	main()



