---
title: Model-based Control
shorttitle: Model-based Control
author: Matthew J. Ellis \href{mailto:mjellis@ucdavis.edu}{mjellis@ucdavis.edu}
shortauthor: M. J. Ellis
date: |
	December 3, 2019 \par
shortdate: Dec. 3, 2019
header-includes:
...

# Introduction

## Background on Model Based Control

### Lecture Notes

- The first part of the lecture discussing direct synthesis and internal model control is largely based on the book:

> B. A. Ogunnaike and W. H. Ray. *Process Dynamics, Modeling, and Control*. Oxford University Press, 1994.

- The second part of the lecture discussing model predictive control is based on several introductory courses, lectures, and seminars given over the last several years
- PDF of the lecture notes are posted to Canvas
- Markdown source files of these notes may be found **[here](https://github.com/mjellis-cbe/Courses-ECH157/tree/master/Lectures/ModelBasedControl)**

### Fundamental Problems of Process/Systems Analysis

\begin{figure}
	\centering
	\begin{tikzpicture}[scale=0.8, >=Latex]
		% Draw and label box to represent system
		\draw [Cerulean, ultra thick, fill=Cerulean, fill opacity=0.25] (-2, -2) rectangle (2,2);
		\draw [dashed] (-5, 0) -- (8, 0);
		\node [above] at (0,0) {$G_p(s)$};
		\node [below] at (0,0) {$\begin{aligned} \dot{x} & = Ax + Bu \\ y & = Cx \end{aligned}$};

		% Draw and label arrows
		\draw [->, ultra thick] (-4, 0) -- node [pos=0.5, above] {$\hat{u}(s)$} node [pos=0.5, below] {$u(t)$} (-2, 0);
		\draw [->, ultra thick] (2, 0) -- node [pos=0.5, above] {$\hat{y}(s)$} node [pos=0.5, below] {$y(t)$} (4, 0);
		\node [right] at (4,1) {Frequency Domain};
		\node [right] at (4,-1) {Time Domain};
	\end{tikzpicture}
\end{figure}

\pause
1. *Process Dynamics*: Given $u$ and $G_p$, determine $y$
\pause
1. *Process Modeling/Identification*: Given $u$ and $y$, determine $G_p$
\pause
1. *Process Control*: Given $G_p$ and $y$ (desired), determine $u$

### Quintessential Take Away on Process Control

Proportional-Integral Controller
$$
	u(s) = K_c \onslide<2->{ \left( 1 + \frac{1}{\tau_I s} \right) }
$$
\begin{itemize}
	\onslide<2->{\item P-term: speed of response}
	\onslide<3>{\item I-term: remove offset}
\end{itemize}

### Process Control Synthesis/Design

- Model-based approach
	- Explicitly utilize a model of the process to design the controller
	- Approaches
		- Direct synthesis approach: desired output specified as a trajectory
		- Optimization approach: desired output specified as a performance index
\pause
- Non-model-based approach (sometimes referred to as model-free approach)
	- Determine controller/parameters from process data
	- Still requires insight into the process being controlled

# Direct Synthesis Approaches

## Direct Synthesis Control

### Direct Synthesis Control

\begin{figure}
	\centering
	\begin{tikzpicture}[
		>=latex',
		block/.style={
			draw,
			rectangle,
			text centered,
			minimum height=2.5em,
			text width=4em,
			rounded corners,
			node distance=8em,
			fill=ucd-blue!50},
		sum/.style={
			draw,
			circle,
			node distance=4em}]

		\node [sum] (sum1) {};
		\node [block, right of=sum1, node distance=6em] (gc) {$G_c(s)$};
		\node [block, right of=gc] (gp) {$G_p(s)$};
		\draw [->] ($(sum1.west)+(-1,0)$) -- node [pos=0.2, above] {$y_{d}$} node [pos=0.8, above] {$+$} (sum1);
		\draw [->] (sum1) -- node [pos=0.5, above] {$e$} (gc);
		\draw [->] (gc) -- node [pos=0.5, above] {$u$} (gp);
		\draw [->] (gp) -- node [pos=0.5] (fdk) {} node [pos=0.7, above] {$y$} ($(gp.east)+(1,0)$);
		\draw [->] (fdk |- gp) -- ($(fdk)+(0,-1)$) -| node [pos=0.9, left] {$-$} (sum1);
	\end{tikzpicture}
\end{figure}

- Closed-loop transfer function
\pause
$$
	y(s) = \frac{G_p G_c}{1+G_pG_c} y_d(s)
$$
\pause
- Want
$$
	y(s) = q(s) y_d(s)
$$

### Direct Synthesis Control

- Problem: Given $G_p(s)$ and $q(s)$, find $G_c(s)$
- Controller synthesis formula
\pause
$$
	G_c(s) = \frac{1}{G_p(s)}\left( \frac{q(s)}{1 - q(s)} \right)
$$
- No assumptions on $G_p$ and $q$ meaning there is no guarantee that resulting controller will be implementable
- Choice of $q$
	\pause
	1. Quick, stable response (little/no overshoot)
	1. Dynamic response which the process is able of achieving
	1. No steady-state offset
	1. Mathematical form as simple as possible

### Direct Synthesis Control

- First-order closed-loop response
$$
	q(s) = \frac{1}{\tau_r s + 1}
$$
- Direct synthesis controller
\pause
$$
	G_c = \frac{1}{\tau_r s} \frac{1}{G_p}
$$
	- Involves inverting $G_p$
	- Be careful when dealing with non-minimum phase processes (e.g., time-delay systems and processes with zeros in right-half plane)

### Direct Synthesis Control

- Examples of low-order processes with first-order closed-loop response
	1. Pure gain process
	$$
		G_p(s) = K, ~G_c(s) = \frac{1}{K\tau_r s}
	$$
	\pause
	$\Rightarrow$ Integral controller
	\pause
	1. Pure capacity process
	$$
		G_p(s) = \frac{K}{s}, ~G_c(s) = \frac{1}{K\tau_r}
	$$
	\pause
	$\Rightarrow$ Proportional controller
	\pause
	1. First-order process
	\pause
	$$
		G_p(s) = \frac{K}{\tau s + 1}, ~G_c(s) = \frac{\tau s + 1}{K\tau_r s}
	$$
	$\Rightarrow$ Proportional-integral (PI) controller with $K_c = \tau/K\tau_r$, $\tau_I = \tau$

### Direct Synthesis Control

- Notes:
	- Controller parameters of standard controllers, i.e., PI control, specified with one parameter, $\tau_r$
	- For low-order systems, direct synthesis leads to controllers that we are familiar and a way to tune the parameters

### Direct Synthesis Control

- Second-order process
$$
	G_p(s) = \frac{K}{\tau^2 s^2 + 2\zeta \tau s + 1}, ~q(s) = \frac{1}{\tau_r s + 1}
$$
- Direct synthesis controller
\pause
$$
	\begin{aligned}
		G_c(s) & = \frac{\tau^2 s^2 + 2 \zeta \tau s + 1}{K \tau_r s} \\
		& = \frac{2\zeta \tau}{K\tau_r}\left(1 + \frac{1}{2\zeta \tau s} + \frac{\tau}{2\zeta} s\right)
	\end{aligned}
$$
$\Rightarrow$ Proportional-integral-derivative (PID) controller with $K_c = 2\zeta \tau / K\tau_r$, $\tau_I = 2\zeta \tau$, $\tau_D = \tau / 2\zeta$

### Direct Synthesis Control

- Inverse of process model
	- $e^{-as}$ term will become $e^{as}$; \pause requires prediction of $y_d$
	\pause
	- Right half complex plane (RHP) zero translates to RHP pole for controller making it unstable
- Time delay process
$$
	G_p(s) = \frac{Ke^{-as}}{\tau s + 1}, ~q(s) = \frac{1}{\tau_r s + 1}
$$
\pause
- Direct synthesis controller
$$
	G_c(s) = \frac{(\tau s + 1)}{K\tau_r s} e^{as}
$$
- Unrealizable since requires predictions of $y_d$

### Direct Synthesis Control

- Time delay process
$$
	G_p(s) = \frac{Ke^{-as}}{\tau s + 1}, ~q(s) = \frac{e^{-a_r s}}{\tau_r s + 1}
$$
- Direct synthesis controller
\pause
$$
	G_c(s) = \frac{(\tau s + 1)}{K} \frac{e^{-(a_r-a)s}}{\tau_r s + 1 - e^{-a_r s}}
$$
- Realizable if $a_r \geq a$

### Time-delay Compensation

- Time-delay process
$$
	G_p(s) = G^*(s) e^{-as}
$$
- Characteristic polynomial
$$
	1 + G_c(s) G^*(s) e^{-as} = 0
$$
	- Increasing time-delay $a$, reduces the upper bound on the controller gain leading to closed-loop stability
	- Smaller controller gain means a slower closed-loop response

### Time-delay Compensation


- Problem: compensate for time-delay to avoid restriction of the controller gain due to the time-delay

\begin{figure}
	\centering
	\begin{tikzpicture}[
		>=latex',
		font=\scriptsize,
		block/.style={
			draw,
			rectangle,
			text centered,
			minimum height=2.5em,
			text width=5em,
			rounded corners,
			node distance=8em,
			fill=ucd-blue!50},
		sum/.style={
			draw,
			circle,
			node distance=3.5em}]

		\node [sum] (sum1) {};
		\node [sum, right of=sum1] (sum2) {};
		\node [block, right of=sum2, node distance=6em] (gc) {$G_c$};
		\node [block, right of=gc] (gp) {$G^*(s)e^{-as}$};
		\node [block, below of=gc, node distance=4em] (gt) {$G_m^*(1-e^{-as})$};

		\draw [->] ($(sum1.west)+(-1,0)$) -- node [pos=0.2, above] {$y_{d}$} node [pos=0.8, above] {$+$} (sum1);
		\draw [->] (gp) -- node [pos=0.5] (fdk) {} node [pos=0.7, above] {$y$} ($(gp.east)+(1,0)$);
		\draw [->] (fdk |- gp) -- ($(fdk)+(0,-2.5)$) -| node [pos=0.95, left] {$-$} (sum1);
		\draw [->] (sum1) -- node [pos=0.5, above] {$e$} node [pos=0.8, above] {$+$} (sum2);
		\draw [->] (sum2) -- node [pos=0.5, above] {$e_c$} (gc);
		\draw [->] (gc) -- node [pos=0.5] (fdk) {} node [pos=0.5, above] {$u$} (gp);
		\draw [->] (fdk |- gp) |- (gt);
		\draw [->] (gt) -| node [pos=0.9,left] {$-$} (sum2);
	\end{tikzpicture}
\end{figure}

- Model of process
$$
	y_m(s) = G_m^*(s) e^{-a_m s} u(s)
$$
- Undelayed process
$$
	\begin{aligned}
		y^*(s) & = G^*(s) u(s) \\
		y^*_m(s) & = G_{m}^* u(s)
	\end{aligned}
$$
	- $y^*(s)$: undelayed version of output

### Time-delay Compensation

- $G_m^* \equiv G^*$ and $a_m = a$ (no modeling errors)
\pause
$$
	\begin{aligned}
		e_c & = e - G^*(1-e^{-as}) u(s) \\
			& = y_d - y(s) - \left[ y^*(s) - y(s) \right] \\
			& = y_d - y^*(s)
	\end{aligned}
$$
- Provide $y^*$ (the prediction of $y$ $a$ time units in the future) to controller
\begin{figure}
	\centering
	\begin{tikzpicture}[
		>=latex',
		block/.style={
			draw,
			rectangle,
			text centered,
			minimum height=2.5em,
			text width=4em,
			rounded corners,
			node distance=7em,
			fill=ucd-blue!50},
		sum/.style={
			draw,
			circle,
			node distance=4em}]

		\node [sum] (sum1) {};
		\node [block, right of=sum1, node distance=6em] (gc) {$G_c(s)$};
		\node [block, right of=gc] (gp) {$G^*(s)$};
		\node [block, right of=gp] (delay) {$e^{-as}$};
		\draw [->] ($(sum1.west)+(-1,0)$) -- node [pos=0.2, above] {$y_{d}$} node [pos=0.8, above] {$+$} (sum1);
		\draw [->] (sum1) -- node [pos=0.5, above] {$e$} (gc);
		\draw [->] (gc) -- node [pos=0.5, above] {$u$} (gp);
		\draw [->] (gp) -- node [pos=0.5] (fdk) {} node [pos=0.5, above] {$y^*$} (delay);
		\draw [->] (delay) -- node [pos=0.7, above] {$y$} ($(delay.east)+(1,0)$);
		\draw [->] (fdk |- gp) -- ($(fdk)+(0,-1)$) -| node [pos=0.9, left] {$-$} (sum1);
	\end{tikzpicture}
\end{figure}
- Eliminate time-delay from feedback loop (time-delay does not affect closed-loop stability)
- Time-delay compensator referred to as Smith predictor
\pause
- Be careful of modeling errors

### Direct Synthesis Controller

- Time delay process (taking $a_r = a$)
$$
	G_p(s) = \frac{Ke^{-as}}{\tau s + 1}, ~q(s) = \frac{e^{-a_r s}}{\tau_r s + 1}
$$
- Direct synthesis controller
$$
	G_c(s) = \frac{(\tau s + 1)}{K} \frac{1}{\tau_r s + 1 - e^{-a s}}
$$
\pause
- Control action
$$
	u(s) = \frac{\tau}{K\tau_r}\left( 1 + \frac{1}{\tau s}\right) \left( e(s) - \left[ y^*(s) - y(s) \right]\right)
$$
- Smith predictor in minor loop with PI controller
- Direct synthesis provides parameters

### Direct Synthesis Controllers

- Specify closed-loop transfer function
- No guarantees that resulting controller is implementable
- Lower-order processes and closed-loop transfer functions, i.e., $q(s)$ yields familiar control laws with explicit formulas for parameters
- Similar approaches may be applied to develop a inverse response compensator and direct synthesis controller for processes with inverse response
- Special treatment for case of RHP zeros (robust stability)

## Internal Model Control

### Internal Model Control

\begin{figure}
	\centering
	\begin{tikzpicture}[
		>=latex',
		block/.style={
			draw,
			rectangle,
			text centered,
			minimum height=2.5em,
			text width=4em,
			rounded corners,
			node distance=8em,
			fill=ucd-blue!50},
		sum/.style={
			draw,
			circle,
			node distance=5em}]

		\node [sum] (sum1) {};
		\node [block, right of=sum1, node distance=6em] (gc) {$G_c(s)$};
		\node [block, right of=gc] (gp) {$G_p(s)$};
		\node [sum, right of=gp] (sum2) {};
		\draw [->] ($(sum1.west)+(-1,0)$) -- node [pos=0.2, above] {$y_{d}$} node [pos=0.8, above] {$+$} (sum1);
		\draw [->] (sum1) -- node [pos=0.5, above] {$e$} (gc);
		\draw [->] (gc) -- node [pos=0.5, above] {$u$} (gp);
		\draw [->] ($(sum2)+(0,1)$) -- node [pos=0.2, right] {$d$} node [pos=0.8, right] {$+$} (sum2);
		\draw [->] (gp) -- node [pos=0.8, above] {$+$} (sum2);
		\draw [->] (sum2) -- node [pos=0.5] (fdk) {} node [pos=0.7, above] {$y$} ($(sum2.east)+(1,0)$);
		\draw [->] (fdk |- gp) -- ($(fdk)+(0,-1)$) -| node [pos=0.9, left] {$-$} (sum1);
	\end{tikzpicture}
\end{figure}

$$
	y(s) = G_p(s) u(s) + d(s)
$$

- Perfect control achieved when $y \equiv y_d$
\pause
- If $G_p(s)$ and $d(s)$ are known, then the controller that to achieve perfect control is
$$
	u(s) = \frac{1}{G_p(s)} \left[ y_d(s) - d(s) \right]
$$

### Internal Model Control

\begin{figure}
	\centering
	\begin{tikzpicture}[
		>=latex',
		block/.style={
			draw,
			rectangle,
			text centered,
			minimum height=2.5em,
			text width=4em,
			rounded corners,
			node distance=8em,
			fill=ucd-blue!50},
		sum/.style={
			draw,
			circle,
			node distance=5em}]

		\node [sum] (sum1) {};
		\node [block, right of=sum1, node distance=6em] (gc) {$G_c(s)$};
		\node [block, right of=gc] (gp) {$G_p(s)$};
		\node [sum, right of=gp] (sum2) {};
		\draw [->] ($(sum1.west)+(-1,0)$) -- node [pos=0.2, above] {$y_{d}$} node [pos=0.8, above] {$+$} (sum1);
		\draw [->] (sum1) -- node [pos=0.5, above] {$e$} (gc);
		\draw [->] (gc) -- node [pos=0.5, above] {$u$} (gp);
		\draw [->] ($(sum2)+(0,1)$) -- node [pos=0.2, right] {$d$} node [pos=0.8, right] {$+$} (sum2);
		\draw [->] (gp) -- node [pos=0.8, above] {$+$} (sum2);
		\draw [->] (sum2) -- node [pos=0.5] (fdk) {} node [pos=0.7, above] {$y$} ($(sum2.east)+(1,0)$);
		\draw [->] (fdk |- gp) -- ($(fdk)+(0,-1)$) -| node [pos=0.9, left] {$-$} (sum1);
	\end{tikzpicture}
\end{figure}

$$
	y(s) = G_p(s) u(s) + d(s)
$$

- In practice, have $G_m(s)$ (model of $G_p(s)$) and $d(s)$ not measured
	1. Estimate disturbance:
	$$
		\hat{d}(s) = y(s) - G_m(s) u(s)
	$$
	1. Let $c(s) = 1/G_m(s)$
\pause
- Internal model control:
$$
	u(s) = c(s) \left[ y_d(s) - \hat{d}(s) \right]
$$

### Internal Model Control

- Interpretation 1:

\vspace{-12pt}
\begin{figure}
	\centering
	\begin{tikzpicture}[
		>=latex',
		font=\scriptsize,
		block/.style={
			draw,
			rectangle,
			text centered,
			minimum height=2.5em,
			text width=4em,
			rounded corners,
			node distance=6em,
			fill=ucd-blue!50},
		sum/.style={
			draw,
			circle,
			node distance=4em}]

		\node [sum] (sum1) {};
		\node [block, right of=sum1] (gc) {$c$};
		\node [block, right of=gc, node distance=8em] (gp) {$G_p$};
		\node [sum, right of=gp, node distance=4em] (sum2) {};
		\node [block, below of=gp, node distance=3em] (gt) {$G_m$};
		\node [sum, right of=gt, node distance=4em] (sum3) {};

		\draw [->] ($(sum1.west)+(-1,0)$) -- node [pos=0.2, above] {$y_{d}$} node [pos=0.8, above] {$+$} (sum1);
		\draw [->] (sum1) -- (gc);
		\draw [->] (gc) -- node [pos=0.5] (fdk) {} node [pos=0.5, above] {$u$} (gp);
		\draw [->] (fdk |- gp) |- (gt);

		\draw [->] (gp) -- node [pos=0.7, below] {$+$} (sum2);
		\draw [->] ($(sum2.north)+(0, 1)$) -- node [pos=0.2, right] {$d$} node [pos=0.8, left] {$+$} (sum2);
		\draw [->] (sum2) -- node [pos=0.5] (fdk) {} node [pos=0.7, above] {$y$} ($(sum2.east)+(1,0)$);
		\draw [->] (fdk |- gp) |- node [pos=0.8, above] {$+$} (sum3);
		\draw [->] (gt) -- node [pos=0.7, above] {$-$} (sum3);
		\draw [->] (sum3) -- ($(sum3)+(0,-1)$) -| node [pos=0.2, above] {$\hat{d}$} node [pos=0.95, left] {$-$} (sum1);
	\end{tikzpicture}
\end{figure}

- Interpretation 2:

\vspace{-12pt}
\begin{figure}
	\centering
	\begin{tikzpicture}[
		>=latex',
		font=\scriptsize,
		block/.style={
			draw,
			rectangle,
			text centered,
			minimum height=2.5em,
			text width=4em,
			rounded corners,
			node distance=6em,
			fill=ucd-blue!50},
		sum/.style={
			draw,
			circle,
			node distance=3.5em}]

		\node [sum] (sum1) {};
		\node [sum, right of=sum1] (sum2) {};
		\node [block, right of=sum2] (gc) {$c$};
		\node [block, right of=gc, node distance=8em] (gp) {$G_p$};
		\node [block, below of=gc, node distance=3em] (gt) {$G_m$};
		\node [sum, right of=gp, node distance=5em] (sum3) {};

		\draw [->] ($(sum1.west)+(-1,0)$) -- node [pos=0.2, above] {$y_{d}$} node [pos=0.8, above] {$+$} (sum1);
		\draw [->] (gp) -- node [pos=0.8, below] {$+$} (sum3);
		\draw [->] ($(sum3.north)+(0,1)$) -- node [pos=0.2, right] {$d$} node [pos=0.8, left] {$+$} (sum3);
		\draw [->] (sum3) -- node [pos=0.5] (fdk) {} node [pos=0.7, above] {$y$} ($(sum3.east)+(1,0)$);
		\draw [->] (fdk |- gp) -- ($(fdk)+(0,-2)$) -| node [pos=0.95, left] {$+$} (sum1);
		\draw [->] (sum1) -- node [pos=0.5, above] {$e$} node [pos=0.8, above] {$+$} (sum2);
		\draw [->] (sum2) -- node [pos=0.5, above] {} (gc);
		\draw [->] (gc) -- node [pos=0.5] (fdk) {} node [pos=0.5, above] {$u$} (gp);
		\draw [->] (fdk |- gp) |- (gt);
		\draw [->] (gt) -| node [pos=0.9, left] {$-$} (sum2);
	\end{tikzpicture}
\end{figure}

### Internal Model Control

- Relationship between conventional feedback control and internal model control
$$
	G_c(s) = \frac{c(s)}{1 - c(s) G_m(s)}, ~c(s) = \frac{G_c(s)}{1 + G_c(s) G_m(s)}
$$
- Closed loop transfer function relations and controller properties
	- Relation of $u$ to $y_d$ and $d$
	$$
		y = \frac{c}{1 + c(G_p-G_m)} (y_d - d)
	$$
	\pause
	- Relation of $y$ to $y_d$ and $d$
	$$
		\begin{aligned}
			y & = d + \frac{G_p c}{1 + c(G_p - G_m)}(y_d - d) \\
			  & = \frac{G_p c}{1 + c(G_p-G_m)} y_d + \frac{1 - G_m c}{1 + c(G_p-G_m)} d
		\end{aligned}
	$$

### Internal Model Control

1. Nominal closed-loop stability ($G_m \equiv G_p$)
$$
	\begin{aligned}
		u & = c(y_d - d) \\
		y & = G_p c(y_d - d) + d
	\end{aligned}
$$
	- Input bounded if $c(s)$ is stable
	- Output bounded if $G_p(s) c(s)$ is stable (requires $G_p(s)$)
1. Offset-free control:
$$
	\begin{aligned}
		\lim_{t \to \infty} y(t) & = \lim_{s \to 0} y(s) \\
		& = \lim_{s \to 0} \frac{G_p(s) c(s)}{D(s)} y_d + \frac{1 - G_m(s) c(s)}{D(s)} d \\
	\end{aligned}
$$
where $D(s) = 1+c(s) (G_p(s) - G_m(s))$
	- If $\lim_{s \to 0} c(s) = 1/G_m(0)$, then $\lim_{s \to 0} 1 - G_m(s) c(s) = 0$ and
	$$
		\lim_{t \to \infty} y(t) = y_d
	$$

### Internal Model Control

- Implementation issues
	1. $G_m$ is never equal to $G_p$ in practices
	1. Implementing $c(s)$ as $1/G_m(s)$ is rarely feasible (time delays, RHP zeros, and infinitely large control action)
\pause
- Design for practical implementation
	1. $G_m = G_{m,+} G_{m,-}$
		- $G_{m,+}$: noninvertible aspects with gain equal to one
		- $G_{m,-}$: invertible parts
	\pause
	1. Controller design and filter design
	$$
		c(s) = \frac{1}{G_{m,-}} f(s), ~f(s) = \frac{1}{(\lambda s + 1)^n}
	$$
	$\lambda$, $n$ selected to ensure that $c(s)$ is proper (numerator order is less than or equal to the denominator order)

### Internal Model Control

- Perfect control is achieved if model is perfect and
$$
	u(s) = \frac{1}{G_p(s)} \left[ y_d(s) - d(s) \right]
$$
- In practice, cannot achieve perfect control, so instead use:
$$
	u(s) = c(s) \left[ y_d(s) - \hat{d}(s) \right]
$$
where $\hat{d}(s) = y(s) - G_m(s) u(s)$

# Optimization-based Control

## Optimization-based Control

### Optimization-based Control

- Model
$$
	\frac{dy}{dt} = f(y,u,d)
$$
- Instead of specifying desired value of $y_d$ along a specified reference trajectory, specify a performance criteria to optimize
\pause
- Objective functional
$$
	\phi = G(y(t_f)) + \int_0^{t_f} F(y,u,d) ~dt
$$
	- $y(t_f)$: output at final time $t$.
	- $G(\cdot)$ and $F(\cdot, \cdot, \cdot)$ chose by control designer to reflect an aspect to be optimized
	\pause
	- May also consider constraints (e.g., valve may only be open 0 to 100 percent)
	$$
		\begin{aligned}
			g(y,u) & \leq 0 \\
			h(y,u) & = 0
		\end{aligned}
	$$

### Multivariable Process Models

- State space model form
	- Nonlinear model
	$$
		\begin{aligned}
			\frac{dx(t)}{dt} & = f(x,u,d) \\
			y(t) & = h(x(t))
		\end{aligned}
	$$
		- $x(t) \in \R^n$: state vector
		- $u(t) \in \R^m$: input vector
		- $y(t) \in \R^l$: output vector
		- $f$ and $h$ are vector valued functions
	\pause
	- Linear model (may be obtained by linearizing the nonlinear model about an operating point)
	$$
		\begin{aligned}
			\dot{x}(t) & = Ax(t) + Bu(t) + B_d d(t) \\
			y(t) & = C x(t)
		\end{aligned}
	$$
		- $A$, $B$, $B_d$, $C$: matrices of appropriate dimensions
		- Note: higher order ordinary differential equations (ODEs) may be converted to a system of first-order ODEs

### Solution of Linear ODEs

- Solution
$$
	\begin{aligned}
		x(t) & = e^{At} x_0 + \int_0^t e^{A(t-\sigma)} B u(\sigma) ~d\sigma \\
		y(t) & = C e^{At} x_0 + C \int_0^t e^{A(t-\sigma)} B u(\sigma) ~d\sigma
	\end{aligned}
$$
- $e^{At}$ denotes the matrix exponential of $At$ (`expm` command in Matlab)
- Pole of system are eigenvalues of $A$

### Digital Control

- Modern control is implemented on digital computers
- Optimization-based control considered here requires solving an optimization problem
\pause
- Need to discretize (in time) the model
	- Partition time into discrete instances called sampling times
	- Let $t_k = \delta k$, $k = 0, 1, 2, \ldots$ and $\delta > 0$ is the sampling period
	- Zeroth order hold of controls - over the sampling period the control action is held constant:
	$$
		u(t) = u_k ~\text{for all} ~t\in [k\delta, (k+1)\delta), ~k = 0,1,\ldots
	$$

### Discrete-time State Space Model

- Model in discrete-time
$$
	\begin{aligned}
		x_{k+1} & = A_d x_k + B_d u_k \\
		y_k & = C x_k
	\end{aligned}
$$
- Matrices
$$
	A_d = e^{A\delta}, ~ B_d = \int_{0}^{\delta} e^{A(t-\sigma)} B~d\sigma
$$
- Use `c2d` command in Matlab
- Note that when $\delta$ is small:
$$
	A_d \approx I + A\delta, ~B_d \approx \delta B
$$
where $I$ is the identity matrix

### Model Predictive Control

- Take $x = 0$ and $u= 0$ to be the desired operating point
- Let $\mathbf{u} = \{ u_0, u_1, \ldots, u_{N-1} \}$

#### Model Predictive Control Problem

$$
	\begin{aligned}
		& \quad \min_{\mathbf{u}} && \sum_{k=0}^{N-1} x_k Q x_k + u_k R u_k \\
		\pause
		& \text{subject to} && x_{k+1} = Ax_k + Bu_k, ~k = 0, \ldots, N-1 \\
		\pause
		&&& x_0 = x \\
		\pause
		&&& x_{lb} \leq x_k \leq x_{ub}, ~k = 1, \ldots, N \\
		&&& u_{lb} \leq u_k \leq u_{ub}, ~k = 0, \ldots, N-1
	\end{aligned}
$$

### Model Predictive Control

- Typically, optimization problem solved on-line every sampling time (requires solving an optimization problem on-line).
- If $Q$ is positive semidefinite and $R$ is positive definite, problem is a convex quadratic program, which very efficient solvers exist

### Model Predictive Control

#### Receding Horizon Implementation

1. Receive a measurement from sensors
1. Solve the MPC problem to obtain the optimal input sequence over the prediction horizon $\{ u^*(0), u^*(1), \ldots, u^*(N-1) \}$
1. Send control action for the first time step to control actuators/lower layer control to implemented over the sampling period
1. Go back to step 1

### Model Predictive Control

![](fig/MPC1.eps "")

### Model Predictive Control

![](fig/MPC2.eps "")

### Model Predictive Control

- Specify a performance criteria that is optimized
- Use a model to make a prediction of future process behavior
- Optimization problem solved on-line to compute the control action
- Receding horizon implementation

### Quintessential Take Away on Process Control

Proportional-Integral Controller
$$
	u(s) = K_c \onslide<2->{ \left( 1 + \frac{1}{\tau_I s} \right) }
$$
\begin{itemize}
	\onslide<2->{\item P-term: speed of response}
	\onslide<3>{\item I-term: remove offset}
\end{itemize}
