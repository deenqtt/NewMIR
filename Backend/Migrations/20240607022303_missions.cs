using System;
using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace Backend.Migrations
{
    /// <inheritdoc />
    public partial class missions : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "Group",
                table: "Missions");

            migrationBuilder.DropColumn(
                name: "Robot",
                table: "Missions");

            migrationBuilder.DropColumn(
                name: "Site",
                table: "Missions");

            migrationBuilder.AlterColumn<string>(
                name: "Name",
                table: "Missions",
                type: "TEXT",
                nullable: false,
                defaultValue: "",
                oldClrType: typeof(string),
                oldType: "TEXT",
                oldNullable: true);

            migrationBuilder.AddColumn<DateTime>(
                name: "CreatedAt",
                table: "Missions",
                type: "TEXT",
                nullable: false,
                defaultValue: new DateTime(1, 1, 1, 0, 0, 0, 0, DateTimeKind.Unspecified));

            migrationBuilder.AddColumn<double>(
                name: "GoalOrientation",
                table: "Missions",
                type: "REAL",
                nullable: false,
                defaultValue: 0.0);

            migrationBuilder.AddColumn<double>(
                name: "GoalX",
                table: "Missions",
                type: "REAL",
                nullable: false,
                defaultValue: 0.0);

            migrationBuilder.AddColumn<double>(
                name: "GoalY",
                table: "Missions",
                type: "REAL",
                nullable: false,
                defaultValue: 0.0);

            migrationBuilder.AddColumn<double>(
                name: "StartOrientation",
                table: "Missions",
                type: "REAL",
                nullable: false,
                defaultValue: 0.0);

            migrationBuilder.AddColumn<double>(
                name: "StartX",
                table: "Missions",
                type: "REAL",
                nullable: false,
                defaultValue: 0.0);

            migrationBuilder.AddColumn<double>(
                name: "StartY",
                table: "Missions",
                type: "REAL",
                nullable: false,
                defaultValue: 0.0);
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "CreatedAt",
                table: "Missions");

            migrationBuilder.DropColumn(
                name: "GoalOrientation",
                table: "Missions");

            migrationBuilder.DropColumn(
                name: "GoalX",
                table: "Missions");

            migrationBuilder.DropColumn(
                name: "GoalY",
                table: "Missions");

            migrationBuilder.DropColumn(
                name: "StartOrientation",
                table: "Missions");

            migrationBuilder.DropColumn(
                name: "StartX",
                table: "Missions");

            migrationBuilder.DropColumn(
                name: "StartY",
                table: "Missions");

            migrationBuilder.AlterColumn<string>(
                name: "Name",
                table: "Missions",
                type: "TEXT",
                nullable: true,
                oldClrType: typeof(string),
                oldType: "TEXT");

            migrationBuilder.AddColumn<string>(
                name: "Group",
                table: "Missions",
                type: "TEXT",
                nullable: true);

            migrationBuilder.AddColumn<string>(
                name: "Robot",
                table: "Missions",
                type: "TEXT",
                nullable: true);

            migrationBuilder.AddColumn<string>(
                name: "Site",
                table: "Missions",
                type: "TEXT",
                nullable: true);
        }
    }
}
