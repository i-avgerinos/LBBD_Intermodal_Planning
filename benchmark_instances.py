from __future__ import division
import sys
import random
import itertools
import time
import math
import datetime
import cplex
from cplex.exceptions import CplexSolverError
from csv import reader
from docplex.cp.model import CpoModel

#Variation 0: 5 Facilities
#Variation 1: 10 Facilities
#Variation 2: 15 Facilities

#5 experiments per variation are run.
print("------------------------------------------------------------------------------------------------------------")
for variation in range(3):
	experiment = 0
	while(experiment != 5):
		##### SECTION 1: GENERATING DATASETS #########################################################################################################################################################
		##### ----------------------------------------------------------------------------------------------------------------------------- ##########################################################
		numFacilities = 5 + variation*5 #Define number of Facilities (Shipment depots and Delivery warehouses)
		failed_attempt = False #'failed_attempt' checks whether the generated dataset implied an infeasible solution or not.
		if failed_attempt == False:
			print("Number of facilities: "+str(numFacilities)+"	|	Variation: "+str(experiment))
			print("--------------------------------------------------------------------------------------------")

		depots = [] #'depots' includes all shipment depots
		warehouses = [] #'warehouses' includes all delivery warehouses

		while(len(depots) < (2/5)*numFacilities or len(warehouses) < (2/5)*numFacilities): #A minimum number of depots and warehouses must be defined.
			depots = []
			warehouses = []
			for j in range(numFacilities):
				counter = random.randint(0, 1) #A binary variable 'counter' is randomly set. 
				if counter == 0:
					depots.append(j) #If 'counter' is 0, a depot is added. 
				if counter == 1:
					warehouses.append(j) #If 'counter' is 1, a warehouse is added.

		#The depots that are served by roadway, railway or seaway are defined. A depot may be served by multiple transport modes.
		road_depots = []
		rail_depots = []
		roro_depots = []
		for j in range(len(depots)):
			#Does depot j allow road transportation?
			answer = random.randint(0, 3)
			#If the answer is 'Yes' (>0), the depot is served by roadway.
			if answer > 0:
				road_depots.append(depots[j])
			#Does depot j allow rail transportation?
			answer = random.randint(0, 3)
			#If the answer is 'Yes' (>0), the depot is served by railway.
			if answer > 0:
				rail_depots.append(depots[j])
			#Does depot j allow RoRo transportation?
			answer = random.randint(0, 3)
			#If the answer is 'Yes' (>0), the depot is served by seadway.
			if answer > 0:
				roro_depots.append(depots[j])


		depots_connections = []
		for j in range(len(depots)):
			depots_connections.append([]) #For each depot, a list of connections is defined. This list may include other depots or warehouses.
		warehousesCovered = [False for l in range(len(warehouses))] #If a warehouse is not connected with any depot, its respective 'warehouseCovered' value is 'False'.

		while(any(warehousesCovered[l] == False for l in range(len(warehouses)))): #All warehouses must be connected with at least one depot.
			for j in range(len(depots)):
				for a in range(len(depots)):
					if a != j:
						#Is depots j connected with depots a?
						answer = random.randint(0, 8)
						if answer == 0:
							depots_connections[j].append(depots[a])
				for b in range(len(warehouses)):
					#Is depots j connected with warehouse b?
					answer = random.randint(0, 8)
					if answer > 0:
						depots_connections[j].append(warehouses[b])
						warehousesCovered[b] = True #If the warehouse 'b' is selected, it is considered as 'Covered'.

		mode_types = ["Roadway", "Railway", "RoRo"]
		fixed_costs = [10*random.randint(1, 10 + random.randint(0, experiment + 1)*10), 10*random.randint(1, 5 + random.randint(0, experiment + 1)*10), 10*random.randint(1, 5 + random.randint(0, experiment + 1)*5)] #Define the fixed cost of each type of mode.
		capacities = [random.randint(5, 5 + experiment), random.randint(5, 10 + experiment), random.randint(5, 15 + experiment)] #Define the capacity of each type of mode.
		isHub = [] #'isHub' indicates whether a warehouse is a hub (in which re-consolidation of orders is allowed) or not.
		for l in range(len(warehouses)):
			#Is warehouse 'l' a hub?
			answer = random.randint(0, 1)
			if answer == 1:
				isHub.append(True)
			else:
				isHub.append(False)

		mode_code = [] #ID of the mode
		departure = [] #Departure time
		arrival = [] #Arrival time
		origin = [] #Origin point
		destination = [] #Destination point
		Fixedcost = [] #Fixed cost
		Trailercost = [] #Cost per used trailer
		Capacity = [] #Capacity in number of trailers

		counter = 0

		for d in range(6):
			for j in range(len(road_depots)):
				#Is depot j served by roadway in day d?
				answer = random.randint(0, 1)
				if answer > 0:
					for k in range(len(depots)):
						if depots[k] == road_depots[j]:
							for l in range(len(depots_connections[k])):
								if road_depots[j] != depots_connections[k][l]:
									mode_code.append(counter)
									counter = counter + 1
									time_1 = 24*d + 8
									time_2 = 5*random.randint(1, experiment + random.randint(1, 3)*4)
									departure.append(time_1)
									arrival.append(time_1 + time_2)
									origin.append(road_depots[j])
									destination.append(depots_connections[k][l])
									Fixedcost.append(fixed_costs[0])
									Trailercost.append(time_2)
									Capacity.append(capacities[0])
			for j in range(len(rail_depots)):
				#Is depot j served by roadway in day d?
				answer = random.randint(0, 1)
				if answer > 0:
					for k in range(len(depots)):
						if depots[k] == rail_depots[j]:
							for l in range(len(depots_connections[k])):
								if rail_depots[j] != depots_connections[k][l]:
									mode_code.append(counter)
									counter = counter + 1
									time_1 = 24*d + 8
									time_2 = 5*random.randint(1, 2*experiment + random.randint(1, 3)*8)
									departure.append(time_1)
									arrival.append(time_1 + time_2)
									origin.append(rail_depots[j])
									destination.append(depots_connections[k][l])
									Fixedcost.append(fixed_costs[1])
									Trailercost.append(time_2)
									Capacity.append(capacities[1])
			for j in range(len(roro_depots)):
				#Is depot j served by roadway in day d?
				answer = random.randint(0, 1)
				if answer > 0:
					for k in range(len(depots)):
						if depots[k] == roro_depots[j]:
							for l in range(len(depots_connections[k])):
								if roro_depots[j] != depots_connections[k][l]:
									mode_code.append(counter)
									counter = counter + 1
									time_1 = 24*d + 8
									time_2 = 5*random.randint(1, 3*experiment + random.randint(1, 3)*12)
									departure.append(time_1)
									arrival.append(time_1 + time_2)
									origin.append(roro_depots[j])
									destination.append(depots_connections[k][l])
									Fixedcost.append(fixed_costs[2])
									Trailercost.append(time_2)
									Capacity.append(capacities[2])

		instance_orders = [] #'instance_orders' includes all delivery requests
		depot_of_order = []
		counter = numFacilities
		for j in range(len(depots)):
			depot_of_order.append([]) #Each depot is associated with a list in 'depot_of_order'
			numOrders = random.randint(10, 80) #A number between 10 and 80 orders is served by each depot.
			for n in range(numOrders):
				depot_of_order[j].append(counter) #Each created order is assigned to a depot.
				instance_orders.append([counter, 100*random.randint(10, 100), 24*random.randint(7, 12), random.randint(10, 100)]) #Each order n has an ID (instance_orders[n][0], a Demand (instance_orders[n][1]), a Deadline (instance_orders[n][2]) and an Importance value (instance_orders[n][3])).
				counter = counter + 1

		disposition_code = [] #ID of the disposition route
		disposition_origin = [] #The depot that serves the disposition route
		disposition_destination = [] #Identical with the 'disposition_origin' depot
		disposition_duration = [] #Duration of the route
		disposition_cost = [] #Cost of the route
		disposition_pw = [] #Demand that is served by the route

		orderCovered = [False for n in range(len(instance_orders))] #Since any order is not served by any route, it is considered as 'False'.

		alpha = []
		for n in range(len(instance_orders)):
			alpha.append([]) #'alpha' includes all disposition routes that served each order.

		counter = 0
		for j in range(len(depots)):
			while(any(orderCovered[n - numFacilities] == False for n in depot_of_order[j])):
				pw = 0
				for n in range(len(depot_of_order[j])):
					#Is order n covered by the route?
					answer = random.randint(1, 50)
					if answer == 1 and pw + instance_orders[depot_of_order[j][n] - numFacilities][1] <= 10000: #The maximum capacity of each trailer is equal to 10000 payweight units.
						pw = pw + instance_orders[depot_of_order[j][n] - numFacilities][1]
						alpha[depot_of_order[j][n] - numFacilities].append(counter) #The newly-created route is added to the 'alpha' list of the order.
						orderCovered[depot_of_order[j][n] - numFacilities] = True
				if pw > 0: #If any order is assigned to the newly created route, the route is defined.
					disposition_code.append(counter)
					disposition_origin.append(depots[j])
					disposition_destination.append(depots[j])
					disposition_cost.append(0.0)
					disposition_duration.append(0.0)
					disposition_pw.append(pw)
					counter = counter + 1

		order_coverage = [] #'order_coverage' includes binary values that define whether an order is served by a route.
		for n in range(len(instance_orders)):
			order_coverage.append([])
			for m in range(len(disposition_code)):
				if disposition_code[m] in alpha[n]:
					order_coverage[n].append(1.0)
				else:
					order_coverage[n].append(0.0)

		for m in range(len(disposition_code)):
			for n in range(len(instance_orders)):
				if order_coverage[n][m] == 1.0:
					disposition_cost[m] = disposition_cost[m] + random.uniform(0.1, 1.5) #Costs and durations of the routes are defined.
			disposition_cost[m] = round(disposition_cost[m], 2)
			disposition_duration[m] = disposition_cost[m]

		trailers = [] #Since the number of trailers is unlimited, we define a number of trailers that is equal to the number of orders.
		for j in range(len(depots)):
			for n in range(len(depot_of_order[j])):
				trailers.append(depots[j]) #Each trailer is fixed to a depot.

		invalid = [] #Define the invalid combinations of modes.
		for m in range(len(mode_code)):
			invalid.append([])
			for k in range(len(mode_code)):
				if k != m:
					if origin[k] == origin[m] and k not in invalid[m]:
						invalid[m].append(k)
					if origin[k] == destination[m] and departure[k] < arrival[m] and k not in invalid[m]:
						invalid[m].append(k)

		transport_costs = []
		for i in range(len(instance_orders) + numFacilities):
			transport_costs.append([])
			for j in range(len(instance_orders) + numFacilities):
				if i == j:
					transport_costs[i].append(0.0)
				else:
					transport_costs[i].append(random.uniform(0.0, 1.5))

		##### ----------------------------------------------------------------------------------------------------------------------------- ##########################################################
		##### END OF SECTION 1 #######################################################################################################################################################################

		##### SECTION 2: MASTER PROBLEM ##############################################################################################################################################################
		##### ----------------------------------------------------------------------------------------------------------------------------- ##########################################################
		start_time = time.time()

		master = cplex.Cplex()
		#master.parameters.workdir.set("/directory_of_user")
		#master.parameters.workmem.set(1024)
		#master.parameters.mip.strategy.file.set(2)
		master.set_results_stream(None)
		master.parameters.timelimit.set(3600)

		master_obj = []
		master_lb = []
		master_ub = []
		master_types = []
		master_names = []

		x_gm = [] #Trailer 'g' to mode 'm' assignment variable
		for g in range(len(trailers)):
			x_gm.append([])
			for m in range(len(mode_code)):
				x_gm[g].append("x_{"+str(g)+","+str(m)+"}")
		for g in range(len(trailers)):
			for m in range(len(mode_code)):
				master_obj.append(0.0)
				master_lb.append(0.0)
				master_ub.append(1.0)
				master_types.append("B")
				master_names.append(x_gm[g][m])

		w_g = [] #1 if trailer 'g' is opened, 0 otherwise
		for g in range(len(trailers)):
			w_g.append("w_{"+str(g)+"}")
			master_obj.append(0.0)
			master_lb.append(0.0)
			master_ub.append(1.0)
			master_types.append("B")
			master_names.append(w_g[g])

		v_m = [] #1 if mode 'm' is opened, 0 otherwise
		for m in range(len(mode_code)):
			v_m.append("v_{"+str(m)+"}")
			master_obj.append(0.0)
			master_lb.append(0.0)
			master_ub.append(1.0)
			master_types.append("B")
			master_names.append(v_m[m])

		a_m = [] #1 if route 'm' is opened, 0 otherwise
		for m in range(len(disposition_code)):
			a_m.append("a_{"+str(m)+"}")
			master_obj.append(0.0)
			master_lb.append(0.0)
			master_ub.append(1.0)
			master_types.append("B")
			master_names.append(a_m[m])

		y_ng = [] #Order 'n' to trailer 'g' assignment variable
		for n in range(len(instance_orders)):
			y_ng.append([])
			for g in range(len(trailers)):
				y_ng[n].append("y_{"+str(n)+","+str(g)+"}")
		for n in range(len(instance_orders)):
			for g in range(len(trailers)):
				validation_test = False #Not all orders-to-trailers assignments are eligible.
				for j in range(len(depots)):
					if n + numFacilities in depot_of_order[j] and trailers[g] == depots[j]: #If trailer 'g' is located to depot 'j' and order 'n' is served by this depot, the assignment is eligible.
						validation_test = True
						break
				if validation_test == True:
					master_obj.append(0.0)
					master_lb.append(0.0)
					master_ub.append(1.0)
					master_types.append("B")
					master_names.append(y_ng[n][g])
				if validation_test == False: #Otherwise, the variable is fixed to 0.
					master_obj.append(0.0)
					master_lb.append(0.0)
					master_ub.append(0.0)
					master_types.append("B")
					master_names.append(y_ng[n][g])

		h_nm = [] #Order 'n' to mode 'm' assignment variable
		for n in range(len(instance_orders)):
			h_nm.append([])
			for m in range(len(mode_code)):
				h_nm[n].append("h_{"+str(n)+","+str(m)+"}")
		for n in range(len(instance_orders)):
			for m in range(len(mode_code)):
				master_obj.append(0.0)
				master_lb.append(0.0)
				master_ub.append(1.0)
				master_types.append("B")
				master_names.append(h_nm[n][m])

		load_g = [] #The load of trailer 'g'
		for g in range(len(trailers)):
			load_g.append("load_{"+str(g)+"}")
			master_obj.append(0.0)
			master_lb.append(0.0)
			master_ub.append(10000.0) #The maximum capacity of each trailer is 10000 payweight units.
			master_types.append("C")
			master_names.append(load_g[g])

		T_n = [] #Variables for Relaxation R_2, indicating the squared number of days of delayed delivery of order 'n'
		for n in range(len(instance_orders)):
			T_n.append("T_{"+str(n)+"}")
			master_obj.append(0.0)
			master_lb.append(0.0)
			master_ub.append(400.0) #The maximum number of days of delay is 20 (20^2 = 400).
			master_types.append("I")
			master_names.append(T_n[n])

		z_objective = ["z"] #The objective function 'z'
		master_obj.append(1.0)
		master_lb.append(0.0)
		master_ub.append(cplex.infinity) #The objective function is unbounded.
		master_types.append("C")
		master_names.append(z_objective[0])

		master.variables.add(obj = master_obj,
							 lb = master_lb,
						 	 ub = master_ub,
							 types = master_types,
							 names = master_names)

		master_expressions = []
		master_senses = []
		master_rhs = []

		#Constraints (1)
		for n in range(len(instance_orders)):
			c1 = cplex.SparsePair(ind = [a_m[m] for m in range(len(disposition_code))],
								  val = [order_coverage[n][m] for m in range(len(disposition_code))])
			master_expressions.append(c1)
			master_senses.append("E")
			master_rhs.append(1.0)
		#Constraints (2)
		for n in range(len(instance_orders)):
			c2 = cplex.SparsePair(ind = [y_ng[n][g] for g in range(len(trailers))],
								  val = [1.0]*len(trailers))
			master_expressions.append(c2)
			master_senses.append("E")
			master_rhs.append(1.0)
		#Constraints (3)
		for g in range(len(trailers)):
			c3 = cplex.SparsePair(ind = [w_g[g]] + [y_ng[n][g] for n in range(len(instance_orders))],
								  val = [10000.0] + [-instance_orders[n][1] for n in range(len(instance_orders))])
			master_expressions.append(c3)
			master_senses.append("G")
			master_rhs.append(0.0)
		#Constraints (4)
		for j in range(len(depots)):
			for g in range(len(trailers)):
				if trailers[g] == depots[j]:
					M_j = []
					for m in range(len(mode_code)):
						if origin[m] == depots[j]:
							M_j.append(m)
					c4 = cplex.SparsePair(ind = [x_gm[g][m] for m in M_j] + [w_g[g]],
										  val = [1.0]*len(M_j) + [-1.0])
					master_expressions.append(c4)
					master_senses.append("E")
					master_rhs.append(0.0)
		#Constraints (5)
		for g in range(len(trailers)):
			for i in range(numFacilities):
				if i not in warehouses and trailers[g] != i:
					M_plus = []
					M_minus = []
					for m in range(len(mode_code)):
						if origin[m] == i:
							M_plus.append(m)
						if destination[m] == i:
							M_minus.append(m)
					c5 = cplex.SparsePair(ind = [x_gm[g][m] for m in M_plus] + [x_gm[g][m] for m in M_minus],
										  val = [1.0]*len(M_plus) + [-1.0]*len(M_minus))
					master_expressions.append(c5)
					master_senses.append("E")
					master_rhs.append(0.0)
		#Constraints (6)
		for g in range(len(trailers)):
			M_l = []
			for l in range(len(warehouses)):
				for m in range(len(mode_code)):
					if destination[m] == warehouses[l]:
						M_l.append(m)
			c6 = cplex.SparsePair(ind = [x_gm[g][m] for m in M_l] + [w_g[g]],
								  val = [1.0]*len(M_l) + [-1.0])
			master_expressions.append(c6)
			master_senses.append("E")
			master_rhs.append(0.0)
		#Constraints (7)
		for g in range(len(trailers)):
			for m in range(len(mode_code)):
				c7 = cplex.SparsePair(ind = [x_gm[g][m]] + [x_gm[g][k] for k in invalid[m]],
									  val = [1.0] + [1.0]*len(invalid[m]))
				master_expressions.append(c7)
				master_senses.append("L")
				master_rhs.append(1.0)
		#Constraints (8)
		for m in range(len(mode_code)):
			c8 = cplex.SparsePair(ind = [v_m[m]] + [x_gm[g][m] for g in range(len(trailers))],
								  val = [Capacity[m]] + [-1.0]*len(trailers))
			master_expressions.append(c8)
			master_senses.append("G")
			master_rhs.append(0.0)
		#Constraints (9)
		for n in range(len(instance_orders)):
			for g in range(len(trailers)):
				for j in range(len(depots)):
					if n in depot_of_order[j] and trailers[g] == depots[j]:
						for m in range(len(mode_code)):
							if origin[m] == depots[j]:
								c9 = cplex.SparsePair(ind = [h_nm[n][m]] + [y_ng[n][g]] + [x_gm[g][m]],
													  val = [1.0] + [-1.0] + [-1.0])
								master_expressions.append(c9)
								master_senses.append("G")
								master_rhs.append(-1.0)
		for n in range(len(instance_orders)):
			for g in range(len(trailers)):
				for j in range(len(depots)):
					if n in depot_of_order[j] and trailers[g] == depots[j]:
						for m in range(len(mode_code)):
							if destination[m] in warehouses:
								c9 = cplex.SparsePair(ind = [h_nm[n][m]] + [y_ng[n][g]] + [x_gm[g][m]],
													  val = [1.0] + [-1.0] + [-1.0])
								master_expressions.append(c9)
								master_senses.append("G")
								master_rhs.append(-1.0)
		#Constraints (10)
		for g in range(len(trailers)):
			c10 = cplex.SparsePair(ind = [load_g[g]] + [y_ng[n][g] for n in range(len(instance_orders))],
								   val = [1.0] + [-instance_orders[n][1] for n in range(len(instance_orders))])	
			master_expressions.append(c10)
			master_senses.append("E")
			master_rhs.append(0.0)
		#Constraints (11)
		for g in range(len(trailers)):
			c11 = cplex.SparsePair(ind = [load_g[g]] + [y_ng[n][g] for n in range(len(instance_orders))],
								   val = [1.0] + [-10000.0 for n in range(len(instance_orders))])	
			master_expressions.append(c11)
			master_senses.append("L")
			master_rhs.append(0.0)
		#Constraints (12)
		for g in range(len(trailers)):
			c12 = cplex.SparsePair(ind = [load_g[g]] + [w_g[g]],
								   val = [1.0] + [-10000.0])	
			master_expressions.append(c12)
			master_senses.append("L")
			master_rhs.append(0.0)
		#Constraints (13)
		c13 = cplex.SparsePair(ind = [load_g[g] for g in range(len(trailers))] + [y_ng[n][g] for n in range(len(instance_orders)) for g in range(len(trailers))],
							   val = [1.0]*len(trailers) + [-instance_orders[n][1] for n in range(len(instance_orders)) for g in range(len(trailers))])
		master_expressions.append(c13)
		master_senses.append("E")
		master_rhs.append(0.0)
		#Constraints (14) replaced by Constraints (45)

		#Constraints (44)
		big_M = 500000
		for n in range(len(instance_orders)):
			for m2 in range(len(mode_code)):
				for m1 in range(len(disposition_code)):
					if order_coverage[n][m1] == 1.0:
						for j in range(len(depots)):
							if n in depot_of_order[j] and origin[m2] == depots[j]:
								c44 = cplex.SparsePair(ind = [a_m[m1]] + [h_nm[n][m2]],
													   val = [big_M] + [big_M])
								master_expressions.append(c44)
								master_senses.append("L")
								master_rhs.append(2*big_M - disposition_duration[m1] + departure[m2])
		#Constraints (45)
		big_M = 500000
		for n in range(len(instance_orders)):
			for g in range(len(trailers)):
				for m in range(len(mode_code)):
					if destination[m] in warehouses:
						c45 = cplex.SparsePair(ind = [T_n[n]] + [x_gm[g][m]] + [y_ng[n][g]],
											  val = [-1.0] + [big_M] + [big_M])
						master_expressions.append(c45)
						master_senses.append("L")
						master_rhs.append(-int(1+((arrival[m]-instance_orders[n][2])/24))*abs(int(1+((arrival[m]-instance_orders[n][2])/24))) + 2*big_M)

		#Constraints (46)
		rhs_var = []
		rhs_par = []
		for g in range(len(trailers)):
			for m in range(len(mode_code)):
				rhs_var.append(x_gm[g][m])
				rhs_par.append(-Trailercost[m])
		for m in range(len(mode_code)):
			rhs_var.append(v_m[m])
			rhs_par.append(-Fixedcost[m])
		for m in range(len(disposition_code)):
			rhs_var.append(a_m[m])
			rhs_par.append(-disposition_cost[m])
		c46 = cplex.SparsePair(ind = [z_objective[0]] + [T_n[n] for n in range(len(instance_orders))] + rhs_var,
							   val = [1.0] + [-instance_orders[n][3] for n in range(len(instance_orders))] + rhs_par)
		master_expressions.append(c46)
		master_senses.append("G")
		master_rhs.append(0.0)

		master.linear_constraints.add(lin_expr = master_expressions, 
									  senses = master_senses,
									  rhs = master_rhs)

		master.objective.set_sense(master.objective.sense.minimize)
		try:
			master.solve()

			lower_bound = round(master.solution.get_objective_value(),1) #The initial lower bound is the objective value of the master problem.

			initial_solution_time = time.time() - start_time
			master_time = round(initial_solution_time, 0)
			print("Initial solution: "+str(lower_bound)+"	in "+str(master_time)+"	|	Gap: "+str(100*round(master.solution.MIP.get_mip_relative_gap(), 4))+" %")
			print("--------------------------------------------------------------------------------------------")

			##### ----------------------------------------------------------------------------------------------------------------------------- ##########################################################
			##### END OF SECTION 2 #######################################################################################################################################################################

			##### SECTION 3: LOGIC-BASED BENDERS DECOMPOSITION ALGORITHM #################################################################################################################################
			##### ----------------------------------------------------------------------------------------------------------------------------- ##########################################################

			convergence = False #While 'convergence' is 'False', the algorithm keeps running.
			iteration = 0
			optimal_solution = 50000000

			while(convergence == False):

				selected_routes = [] #'selected_routes' includes all disposition routes that were selected in the solution of the master problem.
				for m in range(len(disposition_code)):
					if master.solution.get_values(a_m[m]) > 0.9:
						selected_routes.append(disposition_code[m])

				release_times = [] #Indicates the arrival time of each order to a warehouse.
				for n in range(len(instance_orders)):
					min_time = 50000000
					for m in range(len(mode_code)):
						for g in range(len(trailers)):
							for j in range(len(depots)):
								if trailers[g] == depots[j] and origin[m] == depots[j] and n in depot_of_order[j]:
									if departure[m] < min_time and master.solution.get_values(x_gm[g][m]) > 0.9 and master.solution.get_values(y_ng[n][g]) > 0.9:
										min_time = departure[m]
					release_times.append(min_time)
				time_limits = [] #Indicates the deadline of each disposition route.
				for m in range(len(selected_routes)):
					min_time = 5000000
					for n in range(len(instance_orders)):
						if order_coverage[n][selected_routes[m]] > 0 and min_time > release_times[n]:
							min_time = release_times[n]
					time_limits.append(min_time)

				mode_of_order = [] #Indicates the mode that shipped each order to a warehouse.
				for n in range(len(instance_orders)):
					for g in range(len(trailers)):
						for m in range(len(mode_code)):
							if master.solution.get_values(y_ng[n][g]) > 0.9 and master.solution.get_values(x_gm[g][m]) > 0.9 and destination[m] in warehouses:
								mode_of_order.append(m)

				batches_nodes = []
				for l in range(len(warehouses)):
					batches_nodes.append(warehouses[l])
				for n in range(len(instance_orders)):
					batches_nodes.append(instance_orders[n][0])

				t_ij = [] #Transport cost from node 'i' to node 'j'
				for i in range(len(batches_nodes)):
					t_ij.append([])
					for j in range(len(batches_nodes)):
						t_ij[i].append(transport_costs[batches_nodes[i]][batches_nodes[j]])

				batches = [] #Includes fixed sequences of nodes
				batches_times = [] #Includes the arrival times of all nodes in each sequence
				batches_fixed = [] #Includes the fixed cost of the batch
				batches_release = [] #Includes the earliest start time of each batch
				batches_origin = [] #Includes the origin warehouse of the batch
				for m in range(len(mode_code)):
					if master.solution.get_values(v_m[m]) > 0.9 and destination[m] in warehouses:
						for l in range(len(warehouses)):
							if destination[m] == warehouses[l] and isHub[l] == True:
								list_of_nodes = []
								for g in range(len(trailers)):
									for n in range(len(instance_orders)):
										if master.solution.get_values(x_gm[g][m]) > 0.9 and master.solution.get_values(y_ng[n][g]) > 0.9:
											list_of_nodes.append(instance_orders[n][0])
								for k in range(1, 4):
									combinations = list(itertools.combinations(list_of_nodes, k))
									for d in range(len(combinations)):
										candidate_batch = []
										deadlines = []
										total_demand = 0
										candidate_batch.append(destination[m])
										deadlines.append(0.0)
										for e in range(len(combinations[d])):
											candidate_batch.append(combinations[d][e])
											deadlines.append(instance_orders[combinations[d][e] - numFacilities][2] + 0.001*instance_orders[combinations[d][e] - numFacilities][3])
											total_demand = total_demand + instance_orders[combinations[d][e] - numFacilities][1]
										candidate_batch = [x for _, x in sorted(zip(deadlines, candidate_batch))]
										if total_demand <= 10000:
											list_of_times = [0.0]
											#final_batch = [x for _, x in sorted(zip(deadlines, candidate_batch))]
											batches.append(candidate_batch)
											for e in range(1, len(candidate_batch)):
												for z in range(len(batches_nodes)):
													if batches_nodes[z] == candidate_batch[e]:
														for v in range(len(batches_nodes)):
															if batches_nodes[v] == candidate_batch[e-1]:
																list_of_times.append(t_ij[v][z])
											batches_times.append(list_of_times)
											batches_fixed.append(sum(list_of_times))
											batches_release.append(arrival[m])
											batches_origin.append(destination[m])

				routes = [] #Includes sequences of nodes (randomly ordered)
				release_of_route = [] #Includes the earliest start time of each batch
				deadlines_of_routes = [] #Includes the deadlines of the orders that are served by each route
				routes_origin = [] #Includes the origin warehouse of the route

				for g in range(len(trailers)):
					if master.solution.get_values(w_g[g]) > 0.9:
						for m in range(len(mode_code)):
							if destination[m] in warehouses and master.solution.get_values(x_gm[g][m]) > 0.9:
								for l in range(len(warehouses)):
									if warehouses[l] == destination[m] and isHub[l] == False:
										one_way = []
										due = []
										one_way.append(destination[m])
										due.append(50000)
										routes_origin.append(destination[m])
										for n in range(len(instance_orders)):
											if master.solution.get_values(y_ng[n][g]) > 0.9:
												for z in range(len(batches_nodes)):
													if batches_nodes[z] == instance_orders[n][0]:
														one_way.append(instance_orders[n][0])
														due.append(instance_orders[n][2])
										routes.append(one_way)
										release_of_route.append(arrival[m])
										deadlines_of_routes.append(due)

				transport_times = []
				for r in range(len(routes)):
					transport_times.append([])
					for i in range(len(routes[r])):
						transport_times[r].append([])
						for j in range(len(routes[r])):
							for k in range(len(batches_nodes)):
								if routes[r][i] == batches_nodes[k]:
									for l in range(len(batches_nodes)):
										if routes[r][j] == batches_nodes[l]:
											transport_times[r][i].append(t_ij[k][l])

				beta = [] #'beta' includes binary values that indicate whether order 'n' is served by a batch 'd'
				for n in range(len(instance_orders)):
					beta.append([])
					for d in range(len(batches)):
						if any(batches[d][e] == n for e in range(1, len(batches[d]))):
							for e in range(1, len(batches[d])):
								if batches[d][e] == n:
									if round(batches_times[d][e],2) > 0:
										beta[n].append(round(batches_times[d][e],2))
										break
									if round(batches_times[d][e],2) == 0:
										beta[n].append(0.0001)
										break
						else:
							beta[n].append(0)

				depots_vehicles = [random.randint(10, 40) for j in range(len(depots))]
				warehouses_vehicles = [random.randint(10, 40) for l in range(len(warehouses))]

				connected_hubs = [] #'connected_hubs' includes the orders that will be re-consolidated in a hub
				for n in range(len(instance_orders)):
					for b in range(len(batches)):
						if beta[n][b] > 0 and n:
							connected_hubs.append(n)
				hub_and_spoke = [] #'hub-and-spoke' includes the orders that are directly transferred to their end-customer, without being re-consolidated
				for n in range(len(instance_orders)):
					for r in range(len(routes)):
						if n in routes[r] and n not in hub_and_spoke:
							hub_and_spoke.append(n)

				master_cost = 0 #'master_cost' is the cost that is implied by the solution of the master problem
				for m in range(len(mode_code)):
					if master.solution.get_values(v_m[m]) > 0.9:
						master_cost = master_cost + Fixedcost[m]
				for m in range(len(mode_code)):
					for g in range(len(trailers)):
						if master.solution.get_values(x_gm[g][m]) > 0.9:
							master_cost = master_cost + Trailercost[m]
				for  m in range(len(disposition_code)):
					if master.solution.get_values(a_m[m]) > 0.9:
						master_cost = master_cost + disposition_cost[m]

				subproblem2 = CpoModel() #S_2 formulation

				pi_m = {} #Interval variables that indicate the start and end time of each selected disposition route
				for m in range(len(selected_routes)):
					start = (0, 72) #The time horizon 'H' of the disposition routes is equal to 72 hours.
					end = (0, 72)
					size = int(disposition_duration[selected_routes[m]]) + 1
					pi_m[(m)] = subproblem2.interval_var(start, end, size, optional = False, name = "pi_"+str(m))
				#Constraints (16)
				for j in range(len(depots)):
					j_list = []
					for m in range(len(selected_routes)):
						if disposition_origin[selected_routes[m]] == depots[j]:
							j_list.append(m)
					subproblem2.add(sum([subproblem2.pulse(pi_m[(m)], 1) for m in j_list]) <= depots_vehicles[j])
				#Constraints (17)
				for n in range(len(instance_orders)):
					for m in range(len(selected_routes)):
						subproblem2.add(subproblem2.if_then((order_coverage[n][selected_routes[m]] == 1.0), (subproblem2.end_of(pi_m[(m)]) <= time_limits[m])))
				phi_b = {} #Interval variables that indicate the start and end time of each selected batch
				for b in range(len(batches)):
					start = ((int(batches_release[b]))*60, 60*(int(max(arrival)) + 96)) #The time horizon 'H' of the batch is equal to the maximum arrival time of modes, added by four days
					end = ((int(batches_release[b]))*60, 60*(int(max(arrival)) + 96))
					phi_b[(b)] = subproblem2.interval_var(start, end, optional = True, name = "phi_"+str(b)) #'phi_b' is optional
				#Constraints (27)
				for n in range(len(connected_hubs)):
					all_batches = []
					for b in range(len(batches)):
						if connected_hubs[n] in batches[b]:
							all_batches.append(b)
					subproblem2.add(sum([subproblem2.presence_of(phi_b[(b)]) for b in all_batches]) == 1)
				o_bi = {} #Interval variables that indicate the start and end time of the delivery of order 'i' in batch 'b'
				for b in range(len(batches)):
					for i in range(len(batches[b])):
						start = ((int(batches_release[b]))*60, 60*(int(max(arrival)) + 96))
						end = ((int(batches_release[b]))*60, 60*(int(max(arrival)) + 96))
						o_bi[(b, i)] = subproblem2.interval_var(start, end, size=60*int(batches_times[b][i]), optional = True, name = "o_"+str(b)+","+str(i))
				#Constraints (28)
				for b in range(len(batches)):
					for i in range(1, len(batches[b])):
						subproblem2.add(subproblem2.presence_of(phi_b[(b)]) == subproblem2.presence_of(o_bi[(b, i)]))
				#Constraints (29)
				for b in range(len(batches)):
					subproblem2.add(subproblem2.start_of(phi_b[(b)]) == subproblem2.start_of(o_bi[(b, 0)]))
				#Constraints (30)
				for b in range(len(batches)):
					subproblem2.add(subproblem2.end_of(phi_b[(b)]) == subproblem2.end_of(o_bi[(b, len(batches[b]) - 1)]))
				#Constraints (31)
				for b in range(len(batches)):
					for i in range(1, len(batches[b])):
						subproblem2.add(subproblem2.if_then((subproblem2.presence_of(o_bi[(b, i)])), (subproblem2.end_of(o_bi[(b, i-1)]) <= subproblem2.start_of(o_bi[(b, i)]))))
				tau_n = [] #Days of delayed delivery
				Tau_n = [] #Squared number of days of delayed delivery
				for n in range(len(instance_orders)):
					tau_n.append(subproblem2.integer_var(0, 20, name = "tau_"+str(n)))
					Tau_n.append(subproblem2.integer_var(0, 400, name = "Tau_"+str(n)))
				#Constraints (32)
				for b in range(len(batches)):
					for i in range(1, len(batches[b])):
						subproblem2.add(subproblem2.if_then((subproblem2.presence_of(o_bi[(b, i)])), (tau_n[batches[b][i] - numFacilities] >= (subproblem2.end_of(o_bi[(b, i)])/60 - instance_orders[batches[b][i] - numFacilities][2])/24)))
				#Constraints (33)
				for n in range(len(instance_orders)):
					subproblem2.add(Tau_n[n] == tau_n[n]*tau_n[n])
				#Constraints (34) is neglected, because we do not consider any non-operating hours for the randomly generated datasets.

				#Constraints (35)
				for l in range(len(warehouses)):
					l_batches = []
					for b in range(len(batches)):
						if batches_origin[b] == warehouses[l]:
							l_batches.append(b)
					subproblem2.add(sum([subproblem2.pulse(phi_b[(l_batches[b])], 1) for b in range(len(l_batches))]) <= warehouses_vehicles[l])

				total_cost = sum([(subproblem2.length_of(phi_b[(b)])) for b in range(len(batches))])/60 + sum([Tau_n[n]*instance_orders[n][3] for n in range(len(instance_orders))])
				subproblem2.add(subproblem2.minimize(total_cost))

				start_time = time.time()
				s2_sol = subproblem2.solve(agent = 'local', execfile='/opt/ibm/ILOG/CPLEX_Studio201/cpoptimizer/bin/x86-64_linux/cpoptimizer', TimeLimit = 900, trace_log = False)

				end_time = time.time()
				sub_time = round(end_time - start_time, 0)

				if s2_sol: #If S_2 is feasible, we proceed to S_1.
					subproblem1 = cplex.Cplex()

					subproblem1.parameters.workdir.set("/home/avgerinosi/Benders/NodeFiles")
					subproblem1.parameters.workmem.set(1024)
					subproblem1.parameters.mip.strategy.file.set(2)
					subproblem1.set_results_stream(None)
					subproblem1.parameters.timelimit.set(900)

					subproblem1_obj = []
					subproblem1_lb = []
					subproblem1_ub = []
					subproblem1_types = []
					subproblem1_names = []

					delta_rij = [] #Binary variables that indicate whether node 'j' succeeds node 'i' in route 'r' or not
					for r in range(len(routes)):
						delta_rij.append([])
						for i in range(len(routes[r])):
							delta_rij[r].append([])
							for j in range(len(routes[r])):
								delta_rij[r][i].append("delta_{"+str(r)+","+str(i)+","+str(j)+"}")
					for r in range(len(routes)):
						for i in range(len(routes[r])):
							for j in range(len(routes[r])):
								if i != j:
									subproblem1_obj.append(0.0)
									subproblem1_lb.append(0.0)
									subproblem1_ub.append(1.0)
									subproblem1_types.append("B")
									subproblem1_names.append(delta_rij[r][i][j])
								if i == j: #A node cannot succeed itself.
									subproblem1_obj.append(0.0)
									subproblem1_lb.append(0.0)
									subproblem1_ub.append(0.0)
									subproblem1_types.append("B")
									subproblem1_names.append(delta_rij[r][i][j])

					C_ri = [] #The delivery time of order 'i' in route 'r'
					for r in range(len(routes)):
						C_ri.append([])
						for i in range(len(routes[r])):
							C_ri[r].append("C_{"+str(r)+","+str(i)+"}")
					for r in range(len(routes)):
						for i in range(len(routes[r])):
							subproblem1_obj.append(0.0)
							subproblem1_lb.append(0.0)
							subproblem1_ub.append(cplex.infinity)
							subproblem1_types.append("C")
							subproblem1_names.append(C_ri[r][i])

					Cmax_r = [] #The completion time of route 'r'
					for r in range(len(routes)):
						Cmax_r.append("Cmax_{"+str(r)+"}")
						subproblem1_obj.append(1.0)
						subproblem1_lb.append(0.0)
						subproblem1_ub.append(cplex.infinity)
						subproblem1_types.append("C")
						subproblem1_names.append(Cmax_r[r])

					delay_rik = [] #Binary variables that indicate whether order 'i' of route 'r' is delayed by 'k' days or not
					for r in range(len(routes)):
						delay_rik.append([])
						for i in range(len(routes[r])):
							delay_rik[r].append([])
							for k in range(20):
								delay_rik[r][i].append("delay_{"+str(r)+","+str(i)+","+str(k)+"}")
					for r in range(len(routes)):
						for i in range(len(routes[r])):
							for k in range(20):
								if i != 0:
									subproblem1_obj.append(instance_orders[routes[r][i] - numFacilities][3]*k*k)
									subproblem1_lb.append(0.0)
									subproblem1_ub.append(1.0)
									subproblem1_types.append("B")
									subproblem1_names.append(delay_rik[r][i][k])
								if i == 0: #Node 0 is the warehouse, that implies no delay costs.
									subproblem1_obj.append(0.0)
									subproblem1_lb.append(0.0)
									subproblem1_ub.append(1.0)
									subproblem1_types.append("B")
									subproblem1_names.append(delay_rik[r][i][k])
					#Variables 'day_rit' are neglected, because we do not consider any non-operating hours.

					subproblem1.variables.add(obj = subproblem1_obj,
											  lb = subproblem1_lb,
											  ub = subproblem1_ub,
											  types = subproblem1_types,
											  names = subproblem1_names)

					subproblem1_expressions = []
					subproblem1_senses = []
					subproblem1_rhs = []

					#Constraints (18)
					for r in range(len(routes)):
						for i in range(len(routes[r])):
							c18 = cplex.SparsePair(ind = [delta_rij[r][i][j] for j in range(len(routes[r]))],
												   val = [1.0]*len(routes[r]))
							subproblem1_expressions.append(c18)
							subproblem1_senses.append("E")
							subproblem1_rhs.append(1.0)
					#Constraints (19)
					for r in range(len(routes)):
						for i in range(len(routes[r])):
							c19 = cplex.SparsePair(ind = [delta_rij[r][j][i] for j in range(len(routes[r]))],
												   val = [1.0]*len(routes[r]))
							subproblem1_expressions.append(c19)
							subproblem1_senses.append("E")
							subproblem1_rhs.append(1.0)
					#Constraints (20)
					big_M = 50000
					for r in range(len(routes)):
						for i in range(len(routes[r])):
							for j in range(1, len(routes[r])):
								if i != j:
									c20 = cplex.SparsePair(ind = [C_ri[r][j]] + [C_ri[r][i]] + [delta_rij[r][i][j]],
														   val = [1.0] + [-1.0] + [-big_M])
									subproblem1_expressions.append(c20)
									subproblem1_senses.append("G")
									subproblem1_rhs.append(transport_times[r][i][j] - big_M)
					#Constraints (21)
					for r in range(len(routes)):
						for i in range(len(routes[r])):
							c21 = cplex.SparsePair(ind = [delay_rik[r][i][k] for k in range(20)],
												   val = [1.0]*20)
							subproblem1_expressions.append(c21)
							subproblem1_senses.append("E")
							subproblem1_rhs.append(1.0)
					#Constraints (22)
					for r in range(len(routes)):
						for i in range(1, len(routes[r])):
							c22 = cplex.SparsePair(ind = [delay_rik[r][i][k] for k in range(20)] + [C_ri[r][i]],
												   val = [k for k in range(20)] + [-(1/24)])
							subproblem1_expressions.append(c22)
							subproblem1_senses.append("G")
							subproblem1_rhs.append(-(instance_orders[routes[r][i] - numFacilities][2]/24) + (release_of_route[r]/24))
					#Constraints (23)
					for r in range(len(routes)):
						for i in range(len(routes[r])):
							c23 = cplex.SparsePair(ind = [Cmax_r[r]] + [C_ri[r][i]],
												   val = [1.0] + [-1.0])
							subproblem1_expressions.append(c23)
							subproblem1_senses.append("G")
							subproblem1_rhs.append(0.0)
					#Constraints (24)-(26) are neglected.

					subproblem1.linear_constraints.add(lin_expr = subproblem1_expressions, 
													   senses = subproblem1_senses,
													   rhs = subproblem1_rhs)

					subproblem1.objective.set_sense(subproblem1.objective.sense.minimize)
					subproblem1.solve()

					end_time = time.time()
					sub_time = round(end_time - start_time, 0)

					subproblem_cost = 0
					for n in range(len(instance_orders)):
						subproblem_cost = subproblem_cost + s2_sol[Tau_n[n]]*instance_orders[n][3]
					for b in range(len(batches)):
						if len(s2_sol[phi_b[(b)]]) > 0:
							subproblem_cost = subproblem_cost + s2_sol[phi_b[(b)]][2]/60					
					for r in range(len(routes)):
						subproblem_cost = subproblem_cost + subproblem1.solution.get_values(Cmax_r[r])
					for r in range(len(routes)):
						for i in range(1, len(routes[r])):
							for k in range(20):
								if subproblem1.solution.get_values(delay_rik[r][i][k]) > 0.9:
									subproblem_cost = subproblem_cost + subproblem1.solution.get_values(delay_rik[r][i][k])*instance_orders[routes[r][i] - numFacilities][3]*k*k


					upper_bound = round(subproblem_cost + master_cost,1)
					if upper_bound < optimal_solution:
						optimal_solution = upper_bound
						optimal_bound = lower_bound
					convergence_gap = 0
					if master_time >= 3600:
						convergence_gap = (upper_bound - (lower_bound - master.solution.MIP.get_mip_relative_gap()*lower_bound))/upper_bound
					if master_time < 3600:
						convergence_gap = (upper_bound - lower_bound)/upper_bound
					if convergence_gap <= 0.01 and convergence_gap > 0: #If the Gap is less than 1.0%, the convergence is reached and the algorithm terminates.
						convergence = True
						print("Iteration "+str(iteration)+": "+str(upper_bound)+"	in "+str(sub_time)+"	|	Gap: "+str(100*(round(convergence_gap, 4)))+" %")

					if convergence == False: #If the Gap is greater than 1.0%, the optimality cuts are generated.
						start_time = time.time()

						N_l = [] #'N_l' includes all orders that were assigned to a mode that arrives to warehouse 'l'
						for l in range(len(warehouses)):
							N_l.append([])
							for n in range(len(instance_orders)):
								for g in range(len(trailers)):
									for m in range(len(mode_code)):
										if destination[m] == warehouses[l] and master.solution.get_values(x_gm[g][m]) > 0.9 and master.solution.get_values(y_ng[n][g]) > 0.9:
											N_l[l].append(n)

						S_l = [0 for l in range(len(warehouses))] #'S_l' is the total cost that is implied by warehouse 'l' in the subproblem
						for l in range(len(warehouses)):
							for n in N_l[l]:
								S_l[l] = S_l[l] + s2_sol[Tau_n[n]]*instance_orders[n][3]
							for b in range(len(batches)):
								if batches_origin[b] == warehouses[l] and len(s2_sol[phi_b[(b)]]) > 0:
									S_l[l] = S_l[l] + s2_sol[phi_b[(b)]][2]
							for r in range(len(routes)):
								if warehouses[l] == routes_origin[r]:
									S_l[l] = S_l[l] + round(subproblem1.solution.get_values(Cmax_r[r]), 2)
									for i in range(1, len(routes[r])):
										for k in range(20):
											if subproblem1.solution.get_values(delay_rik[r][i][k]) > 0.9:
												S_l[l] = S_l[l] + k*k*instance_orders[routes[r][i] - numFacilities][3]

						z_l = [] #'z_l' are the linear integrations of bounding functions 'B_l(h)'
						for l in range(len(warehouses)):
							z_l.append("z_"+str(l)+","+str(iteration))
							master_obj.append(0.0)
							master_lb.append(0.0)
							master_ub.append(cplex.infinity)
							master_types.append("C")
							master_names.append(z_l[l])
						#Constraints (42)
						for l in range(len(warehouses)):
							c42 = cplex.SparsePair(ind = [z_l[l]] + [h_nm[n][mode_of_order[n]] for n in N_l[l]],
													val = [1.0] + [-S_l[l]]*len(N_l[l]))
							master_expressions.append(c42)
							master_senses.append("G")
							master_rhs.append(S_l[l] - S_l[l]*len(N_l[l]))
						#Constraints (43)
						var_list = []
						par_list = []
						for g in range(len(trailers)):
							for m in range(len(mode_code)):
								var_list.append(x_gm[g][m])
								par_list.append(-Trailercost[m])
						for m in range(len(mode_code)):
							var_list.append(v_m[m])
							par_list.append(-Fixedcost[m])
						for m in range(len(disposition_code)):
							var_list.append(a_m[m])
							par_list.append(-disposition_cost[m])
						for l in range(len(warehouses)):
							var_list.append(z_l[l])
							par_list.append(-1.0)
						c43 = cplex.SparsePair(ind = [z_objective[0]] + var_list,
																val = [1.0] + par_list)
						master_expressions.append(c43)
						master_senses.append("G")
						master_rhs.append(0.0)
						
						master = cplex.Cplex()
						#master.parameters.workdir.set("/directory_of_user")
						#master.parameters.workmem.set(1024)
						#master.parameters.mip.strategy.file.set(2)
						master.set_results_stream(None)
						master.parameters.timelimit.set(3600)

						master.variables.add(obj = master_obj,
											lb = master_lb,
											ub = master_ub,
											types = master_types,
											names = master_names)

						master.linear_constraints.add(lin_expr = master_expressions, 
													senses = master_senses,
													rhs = master_rhs)

						master.objective.set_sense(master.objective.sense.minimize)
						master.solve()

						lower_bound = round(master.solution.get_objective_value(),1)
						end_time = time.time()
						master_time = round(end_time - start_time, 0)

						print("Iteration "+str(iteration)+": "+str(upper_bound)+"	in "+str(sub_time + master_time)+"	|	Gap: "+str(100*(round(convergence_gap, 4)))+" %")
				else: #If the subproblem is infeasible, the feasibility cuts are generated.
					start_time = time.time()

					shipment_mode = []
					for n in range(len(instance_orders)):
						for m in range(len(mode_code)):
							for j in range(len(depots)):
								for g in range(len(trailers)):
									if master.solution.get_values(x_gm[g][m]) > 0.9 and master.solution.get_values(y_ng[n][g]) > 0.9 and origin[m] == depots[j] and trailers[g] == depots[j] and n + numFacilities in depot_of_order[j]:
										shipment_mode.append(m)
					num_of_disposition = 0 #Number of disposition route that were selected by the master problem
					for m in range(len(disposition_code)):
						if master.solution.get_values(a_m[m]) > 0.9:
							num_of_disposition = num_of_disposition + 1
					#Constraints (36)
					var_list = []
					par_list = []
					for n in range(len(instance_orders)):
						var_list.append(h_nm[n][shipment_mode[n]])
						par_list.append(1.0)
					for m in range(len(disposition_code)):
						total_orders = 0
						for n in range(len(instance_orders)):
							if order_coverage[n][m] > 0:
								total_orders = total_orders + 1
						var_list.append(a_m[m])
						par_list.append(total_orders)
					c36 = cplex.SparsePair(ind = var_list,
													   val = par_list)
					master_expressions.append(c36)
					master_senses.append("L")
					master_rhs.append(len(instance_orders) + num_of_disposition - 1) #The length of the set of orders and the number of the selected routes is the the length of set 'A^r_x'.

					master = cplex.Cplex()
					#master.parameters.workdir.set("/directory_of_user")
					#master.parameters.workmem.set(1024)
					#master.parameters.mip.strategy.file.set(2)
					master.set_results_stream(None)
					master.parameters.timelimit.set(3600)

					master.variables.add(obj = master_obj,
										lb = master_lb,
										ub = master_ub,
										types = master_types,
										names = master_names)

					master.linear_constraints.add(lin_expr = master_expressions, 
												senses = master_senses,
												rhs = master_rhs)

					master.objective.set_sense(master.objective.sense.minimize)
					master.solve()

					lower_bound = round(master.solution.get_objective_value(),1)
					end_time = time.time()
					master_time = round(end_time - start_time, 0)
					print("Iteration "+str(iteration)+": Infeasible	in "+str(sub_time))
				iteration = iteration + 1
				if iteration == 10: #The maximum number of iterations is 10.
					convergence = True
			experiment = experiment + 1
			##### END OF SECTION 3 #######################################################################################################################################################################
		except CplexSolverError as exc:
			if exc.args[2] == cplex.exceptions.error_codes.CPXERR_NO_SOLN: #If the generated dataset implies infeasibility, the attempt is considered as 'failed' and a new dataset is generated.
				failed_attempt = True
		print("------------------------------------------------------------------------------------------------------------")